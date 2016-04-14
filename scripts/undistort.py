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
        self.max_error = max_error
        with open(lookup_file, 'r') as f:
            self.lookup_dict = yaml.load(f)
            self.lookup_table = {}
            self.sort_lookup_table()
            print "Lookup table ready..."

    def sort_lookup_table(self):
        '''
        Populate lookup data into tables
        '''
        for theta in self.lookup_dict:
            range_table = self.lookup_dict[theta]
            rs = range_table.keys()
            errors = []
            for r in range_table:
                e = range_table[r]
                gt = r - e
                error = gt/r
                errors.append(error)
            fc = None
            fl = None
            if len(rs) > 3:
                fc = interpolate.interp1d(rs, errors, kind='cubic')
#                fl = interpolate.interp1d(rs, errors, kind='linear',
#                bounds_error=False, fill_value="extrapolate")
            else:
                continue
            self.lookup_table[theta] = (fc, fl)

    def fill_gap(self, theta):
        best_key = float('inf')
        for key in self.lookup_table.keys():
            if best_key > abs(key - theta):
                best_key = key
        self.lookup_table[theta] = self.lookup_table[best_key]

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
            if theta not in self.lookup_table:
                self.fill_gap(theta)
            (fc, fl) = self.lookup_table[theta]
            error = 0.0
            try:
                error = fc(beam)
            except:
                maxr = max(self.lookup_table[keys].keys())
                error =  self.lookup_table[keys][maxr]
            if abs(error) > self.max_error:
                filtered_scan.ranges.append(beam)
            else:
                filtered_scan.ranges.append(error*beam)
        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node("undistort_scan")
    rospack = rospkg.RosPack()
    path = rospack.get_path('rplidar_ros')
    config_path = os.path.join(path, 'config', 'calibration_lookup.yml')
    lookup_table = LookupUndistort(config_path)
    rospy.Subscriber('/raw_scan', LaserScan, lookup_table.laser_cb)
    rospy.spin()
