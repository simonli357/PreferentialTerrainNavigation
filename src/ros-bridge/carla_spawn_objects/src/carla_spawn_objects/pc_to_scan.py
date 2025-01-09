#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import math

# LiDAR configuration (based on the provided settings)
LIDAR_RANGE = 50.0  # Maximum range of the LiDAR
VERTICAL_UPPER_FOV = 2.0  # Upper field of view (degrees)
VERTICAL_LOWER_FOV = -26.8  # Lower field of view (degrees)

def pointcloud_to_laserscan(pointcloud_msg):
    laser_scan = LaserScan()
    laser_scan.header = pointcloud_msg.header
    laser_scan.angle_min = -math.pi  # Start angle (-180 degrees)
    laser_scan.angle_max = math.pi   # End angle (180 degrees)
    laser_scan.angle_increment = 0.01  # Resolution (in radians)
    laser_scan.range_min = 0.1  # Minimum valid range
    laser_scan.range_max = LIDAR_RANGE  # Maximum range from config

    # Initialize ranges with "inf"
    num_ranges = int((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment)
    laser_scan.ranges = [float('inf')] * num_ranges

    for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point[0], point[1], point[2]

        # Filter points by vertical angle (Z) based on FOV
        vertical_angle = math.degrees(math.atan2(z, math.sqrt(x**2 + y**2)))
        if vertical_angle > VERTICAL_UPPER_FOV or vertical_angle < VERTICAL_LOWER_FOV:
            continue

        # Convert to polar coordinates (2D plane)
        range_val = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)

        # Check if the angle is within the scan range
        if laser_scan.angle_min <= angle <= laser_scan.angle_max:
            index = int((angle - laser_scan.angle_min) / laser_scan.angle_increment)

            # Safeguard against index out-of-range errors
            if 0 <= index < num_ranges:
                # Update range if closer
                if range_val < laser_scan.ranges[index]:
                    laser_scan.ranges[index] = range_val

    return laser_scan

def pointcloud_callback(msg):
    try:
        laser_scan = pointcloud_to_laserscan(msg)
        laser_scan_pub.publish(laser_scan)
    except Exception as e:
        rospy.logerr(f"Error in pointcloud_callback: {e}")

if __name__ == "__main__":
    rospy.init_node("pointcloud_to_laserscan")
    
    pointcloud_sub = rospy.Subscriber("/carla/ego_vehicle/semantic_lidar", PointCloud2, pointcloud_callback)
    laser_scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
    
    rospy.spin()
