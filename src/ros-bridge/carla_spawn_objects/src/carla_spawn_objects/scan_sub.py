#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import math

# subscribe to /scan and print the data
def scan_callback(msg):
    # rospy.loginfo(f"Received scan data: {msg}")
    #print min value
    min_val = min(msg.ranges)
    rospy.loginfo(f"Min range value: {min_val}")
    
if __name__ == "__main__":
    rospy.init_node("scan_sub")
    
    scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback)
    
    rospy.spin()