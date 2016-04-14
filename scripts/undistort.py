#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import LaserScan
import rospkg
import os
from scipy import interpolate


class LookupUndistort(object):

    def __init__(self,
                 lookup_file,
                 k_nearest=1,
                 max_error=0.5,
                 search_region=0.25):
        self.pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        with open(lookup_file, 'r') as f:
            self.lookup_dict = yaml.load(f)
            self.sort_lookup_table()
            self.lookup_table = {}

    def sort_lookup_table(self):
        '''
        Populate lookup data into tables
        '''
        for theta in self.lookup_dict:
            range_table = self.lookup_dict[theta]
            rs = range_table.keys()
            errors = []
            for r in range_table:
                errors.append(range_table[r])
            f = interpolate.interp1d(rs, errors, kind='cubic')
            self.lookup_table[theta] = f

    def idx2radians(self, msg, idx):
        return msg.angle_min + msg.angle_increment*idx

    def laser_cb(self, msg):
        # Copy raw laser msg
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.header.stamp = rospy.get_rostime()
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.intensities = msg.intensities

        for (idx, beam) in enumerate(msg.ranges):
            theta = self.idx2radians(msg, idx)
            fr = self.lookup_table[theta]
            error = fr(beam)
            if abs(error) > self.max_error:
                filtered_scan.ranges.append(beam)
            else:
                filtered_scan.ranges.append(beam-error)
        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node("undistort_scan")
    rospack = rospkg.RosPack()
    path = rospack.get_path('rplidar_ros')
    config_path = os.path.join(path, 'config', 'calibration_lookup.yml')
    lookup_table = LookupUndistort(config_path)
    rospy.Subscriber('/raw_scan', LaserScan, lookup_table.laser_cb)
    rospy.spin()
