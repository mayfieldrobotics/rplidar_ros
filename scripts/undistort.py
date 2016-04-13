#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import LaserScan
import rospkg
import os
import numpy as np
import math
from scipy import spatial


class LookupUndistort(object):

    def __init__(self,
                 lookup_file,
                 k_nearest=4,
                 max_error=0.5,
                 search_region=0.25):
        self.pub = rospy.Publisher("/fscan", LaserScan, queue_size=10)
        self.k_nearest = k_nearest  # Average k-nearest error values

        self.lookup_tree = None  # table of (theta, range) measurements
        self.catesian_lookup_tree = None  # table of (x, y) measurements
        self.max_error = max_error
        self.kdtree = None
        self.search_region = search_region

        with open(lookup_file, 'r') as f:
            self.lookup_table = yaml.load(f)
            self.sort_lookup_table()

    def sort_lookup_table(self):
        '''
        Populate lookup data into tables
        '''
        data = []
        cart_data = []
        for key in self.lookup_table.keys():
            for r in self.lookup_table[key]:
                data.append([key, r])
                cart_data.append([r*math.cos(key), r*math.sin(key)])

        self.lookup_tree = np.array(data)
        self.cartesian_lookup_tree = np.array(cart_data)
        self.kdtree = spatial.KDTree(self.cartesian_lookup_tree)

    def idx2radians(self, msg, idx):
        return msg.angle_min + msg.angle_increment*idx

    def laser_cb(self, msg):
        # Copy raw laser msg
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.intensities = msg.intensities

        for (idx, beam) in enumerate(msg.ranges):
            theta = self.idx2radians(msg, idx)
            x = np.array([beam*math.cos(theta),
                          beam*math.sin(theta)])

            idxs = self.kdtree.query_ball_point(x, self.search_region)
            matches = self.lookup_tree[idxs, :]
            avg_error = 0.0
            num_error = 0
            for match in matches:
                t = match[0]
                r = match[1]
                avg_error += self.lookup_table[t][r]
                num_error += 1
            if num_error == 0 or abs(avg_error) > self.max_error:
                filtered_scan.ranges.append(beam)
            else:
                avg_error = avg_error/num_error
                filtered_scan.ranges.append(beam-avg_error)
        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node("undistort_scan")
    rospack = rospkg.RosPack()
    path = rospack.get_path('rplidar_ros')
    config_path = os.path.join(path, 'config', 'calibration_lookup.yml')
    lookup_table = LookupUndistort(config_path, 2)
    rospy.Subscriber('/scan', LaserScan, lookup_table.laser_cb)
    rospy.spin()
