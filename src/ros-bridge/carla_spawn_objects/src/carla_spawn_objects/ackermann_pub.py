#!/usr/bin/env python33

import rospy
from ackermann_msgs.msg import AckermannDrive

def publish_ackermann_cmd():
    # Initialize the ROS node
    rospy.init_node('ackermann_cmd_publisher', anonymous=True)

    # Create a publisher for the /carla/ego_vehicle/ackermann_cmd topic
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)

    # Set the publish rate to 10 Hz
    rate = rospy.Rate(10)

    # Create an AckermannDrive message with the desired values
    ackermann_cmd = AckermannDrive()
    ackermann_cmd.steering_angle = 0.0
    ackermann_cmd.steering_angle_velocity = 0.0
    ackermann_cmd.speed = 20.0
    ackermann_cmd.acceleration = 0.0
    ackermann_cmd.jerk = 0.0

    rospy.loginfo("Starting to publish AckermannDrive messages to /carla/ego_vehicle/ackermann_cmd at 10 Hz...")

    while not rospy.is_shutdown():
        # Publish the message
        pub.publish(ackermann_cmd)
        rospy.loginfo(f"Published: {ackermann_cmd}")
        
        # Sleep to maintain the 10 Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_ackermann_cmd()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ackermann Command Publisher Node terminated.")
