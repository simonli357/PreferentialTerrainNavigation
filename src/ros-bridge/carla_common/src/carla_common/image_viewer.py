#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import re
import os

# Global variables
current_image = None
current_topic_index = 0
image_topics = [
    "/carla/ego_vehicle/semantic_camera_front/image",
    "/carla/ego_vehicle/semantic_camera_left/image",
    "/carla/ego_vehicle/semantic_camera_right/image",
    "/carla/ego_vehicle/semantic_camera_rear/image",
    "/carla/ego_vehicle/camera_front/image",
    "/carla/ego_vehicle/camera_left/image",
    "/carla/ego_vehicle/camera_right/image",
    "/carla/ego_vehicle/camera_rear/image",
]
bridge = CvBridge()
current_subscriber = None  # Track the current subscriber

# Callback function for receiving images
def image_callback(msg):
    global current_image
    try:
        # Determine the encoding for conversion
        encoding = msg.encoding if hasattr(msg, 'encoding') else "bgr8"
        current_image = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert image on topic {msg._connection_header['topic']}: {e}")
        current_image = None

# Subscribe to the current topic
def subscribe_to_topic(topic):
    global current_subscriber

    # Unsubscribe from the previous topic
    if current_subscriber:
        current_subscriber.unregister()

    # Subscribe to the new topic
    current_subscriber = rospy.Subscriber(topic, Image, image_callback)

# Extract descriptive names from topics
def get_descriptive_name(topic):
    # Match the topic with regex to extract "semantic" or "rgb", and the direction (front, left, etc.)
    if "semantic_camera" in topic:
        match = re.search(r"semantic_camera_(front|left|right|rear)", topic)
        if match:
            direction = match.group(1)
            return f"semantic_{direction}"
    elif "camera" in topic and "image" in topic:
        match = re.search(r"camera_(front|left|right|rear)", topic)
        if match:
            direction = match.group(1)
            return f"rgb_{direction}"
    return "unknown"

# Generate a unique filename in the "images" folder
def generate_filename(base_name):
    folder = "images"
    os.makedirs(folder, exist_ok=True)  # Ensure the folder exists
    index = 1
    while True:
        filename = os.path.join(folder, f"{base_name}_{index}.png")
        if not os.path.exists(filename):
            return filename
        index += 1

# Main function
def main():
    global current_topic_index, current_image

    rospy.init_node("image_viewer", anonymous=True)

    # Subscribe to the initial topic
    subscribe_to_topic(image_topics[current_topic_index])

    rospy.loginfo(f"Subscribed to {image_topics[current_topic_index]}")

    while not rospy.is_shutdown():
        if current_image is not None:
            # Display the image
            cv2.imshow("Image Viewer", current_image)

        # Wait for key press
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # Save the current image with a unique filename
            if current_image is not None:
                descriptive_name = get_descriptive_name(image_topics[current_topic_index])
                filename = generate_filename(descriptive_name)
                cv2.imwrite(filename, current_image)
                rospy.loginfo(f"Image saved as {filename}")
            else:
                rospy.logwarn("No image available to save.")

        elif key == ord('t'):
            # Switch to the next topic
            current_topic_index = (current_topic_index + 1) % len(image_topics)
            subscribe_to_topic(image_topics[current_topic_index])
            rospy.loginfo(f"Switched to {image_topics[current_topic_index]}")

        elif key == 27:  # ESC key
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
