#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import yaml
import numpy as np
import cv2
import argparse

class Camera:
    def __init__(self, config):
        self.K = np.zeros([3, 3])  # K: Camera intrinsic matrix
        self.R = np.zeros([3, 3])  # R: Rotation matrix
        self.t = np.zeros([3, 1])  # t: Translation vector
        self.P = np.zeros([3, 4])  # P: Projection matrix
        self._initialize(config)

    def _initialize(self, config):
        self.setK(config["fx"], config["fy"], config["px"], config["py"])  # Set intrinsic parameters
        self.setR(np.deg2rad(config["yaw"]), np.deg2rad(config["pitch"]), np.deg2rad(config["roll"]))  # Set rotation
        self.setT(config["XCam"], config["YCam"], config["ZCam"])  # Set translation
        self.updateP()  # Update projection matrix

    def setK(self, fx, fy, px, py):
        self.K[0, 0] = fx  # Focal length in x direction
        self.K[1, 1] = fy  # Focal length in y direction
        self.K[0, 2] = px  # Principal point x-coordinate
        self.K[1, 2] = py  # Principal point y-coordinate
        self.K[2, 2] = 1.0  # Scale factor

    def setR(self, yaw, pitch, roll):
        # Rotation matrices around each axis
        Rz = np.array([[np.cos(-yaw), -np.sin(-yaw), 0.0],  # Rotation around z-axis
                       [np.sin(-yaw),  np.cos(-yaw), 0.0],
                       [0.0,           0.0,          1.0]])
        Ry = np.array([[np.cos(-pitch), 0.0, np.sin(-pitch)],  # Rotation around y-axis
                       [0.0,            1.0, 0.0],
                       [-np.sin(-pitch), 0.0, np.cos(-pitch)]])
        Rx = np.array([[1.0,    0.0,           0.0],  # Rotation around x-axis
                       [0.0,    np.cos(-roll), -np.sin(-roll)],
                       [0.0,    np.sin(-roll),  np.cos(-roll)]])
        # Switch axes (x = -y, y = -z, z = x)
        Rs = np.array([[0.0, -1.0, 0.0],  # Switching axes for camera convention
                       [0.0,  0.0, -1.0],
                       [1.0,  0.0, 0.0]])

        self.R = Rs.dot(Rz.dot(Ry.dot(Rx)))  # Combined rotation matrix

    def setT(self, XCam, YCam, ZCam):
        X = np.array([XCam, YCam, ZCam])  # Camera position in world coordinates
        self.t = -self.R.dot(X)  # Translation vector in camera frame

    def updateP(self):
        Rt = np.zeros([3, 4])  # Combined rotation and translation matrix
        Rt[0:3, 0:3] = self.R  # Add rotation matrix
        Rt[0:3, 3] = self.t.ravel()  # Add translation vector
        self.P = self.K.dot(Rt)  # Compute projection matrix

def load_camera_configs_and_images(camera_config_paths, image_paths, batch=False):
    """
    Load multiple camera configurations and corresponding images.
    If batch=True, image_paths are directories and all images in those directories will be processed.
    """
    cameraConfigs = []
    for config_path in camera_config_paths:
        with open(os.path.abspath(config_path)) as stream:
            cameraConfigs.append(yaml.safe_load(stream))

    if not batch:
        # Single set of images
        imagePaths = [image_paths]
        outputFilenames = [os.path.basename(image_paths[0])]
    else:
        # Process directories of images
        all_images = []
        for p in image_paths:
            images_in_dir = [os.path.join(p, f) for f in sorted(os.listdir(p)) if f[0] != "."]
            all_images.append(images_in_dir)
        # Transpose so that we get tuples of corresponding frames
        imagePaths = list(map(list, zip(*all_images)))
        outputFilenames = [os.path.basename(imgs[0]) for imgs in imagePaths]

    print("cameraConfigs:", cameraConfigs)
    print("outputFilenames:", outputFilenames)
    return cameraConfigs, imagePaths, outputFilenames


def compute_masks(cameraConfigs, outputRes, droneConfig, pxPerM):
    """
    Compute masks to clip invalid parts of transformed images.
    Vectorized approach to determine mask pixels.
    """
    masks = []
    # Precompute grid
    h, w = outputRes
    j_coords, i_coords = np.indices((h, w))
    # Calculate angles (theta)
    # theta = arctan2(-j + offset_y, i - offset_x)
    offset_y = outputRes[0]/2.0 - droneConfig["YCam"] * pxPerM[0]
    offset_x = -outputRes[1]/2.0 + droneConfig["XCam"] * pxPerM[1]
    theta_radians = np.arctan2(-(j_coords - offset_y), (i_coords + offset_x))
    # Convert to degrees
    theta_degrees = np.degrees(theta_radians)

    for config in cameraConfigs:
        # Determine invalid angles (angles behind camera)
        # Condition: abs(theta - yaw) > 90 and abs(theta - yaw) < 270
        yaw = config["yaw"]
        angle_diff = np.abs(theta_degrees - yaw)
        invalid = (angle_diff > 90) & (angle_diff < 270)

        # Expand invalid mask to 3D for image channels
        mask = np.repeat(invalid[:, :, np.newaxis], 3, axis=2)
        masks.append(mask)

    return masks

class IPMNode:
    def __init__(self, gt):
        rospy.init_node('ipm_node', anonymous=True)
        self.bridge = CvBridge()
        if gt:
            self.image_subs = [
                rospy.Subscriber('/carla/ego_vehicle/semantic_camera_front/image', Image, self.callback_front),
                rospy.Subscriber('/carla/ego_vehicle/semantic_camera_left/image', Image, self.callback_left),
                rospy.Subscriber('/carla/ego_vehicle/semantic_camera_rear/image', Image, self.callback_rear),
                rospy.Subscriber('/carla/ego_vehicle/semantic_camera_right/image', Image, self.callback_right)
            ]
        else:
            self.image_subs = [
                rospy.Subscriber('/carla/ego_vehicle/segmentation_front', Image, self.callback_front),
                rospy.Subscriber('/carla/ego_vehicle/segmentation_left', Image, self.callback_left),
                rospy.Subscriber('/carla/ego_vehicle/segmentation_rear', Image, self.callback_rear),
                rospy.Subscriber('/carla/ego_vehicle/segmentation_right', Image, self.callback_right)
            ]
        self.image_pub = rospy.Publisher('/ipm/birds_eye_view', Image, queue_size=10)
        self.images = [None, None, None, None]

    def callback_front(self, msg):
        self.images[0] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_left(self, msg):
        self.images[2] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_rear(self, msg):
        self.images[1] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_right(self, msg):
        self.images[3] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

def main():
    parser = argparse.ArgumentParser(description="Generate Bird's Eye View from multiple camera images")
    parser.add_argument("--gt", action="store_true", help="Use ground truth images")
    parser.add_argument("--show", action="store_true", help="Show images in a window")
    args = parser.parse_args()
    node = IPMNode(args.gt)
    current_dir = os.path.dirname(os.path.realpath(__file__))
    camera_config_paths = [
        os.path.join(current_dir, "camera_configs/1_FRLR/front.yaml"),
        os.path.join(current_dir, "camera_configs/1_FRLR/rear.yaml"),
        os.path.join(current_dir, "camera_configs/1_FRLR/left.yaml"),
        os.path.join(current_dir, "camera_configs/1_FRLR/right.yaml")
    ]
    image_dirs = [
        os.path.join(current_dir, "../data/1_FRLR/carla/front"),
        os.path.join(current_dir, "../data/1_FRLR/carla/rear"),
        os.path.join(current_dir, "../data/1_FRLR/carla/left"),
        os.path.join(current_dir, "../data/1_FRLR/carla/right")
    ]
    drone_config_path = os.path.join(current_dir, "camera_configs/1_FRLR/drone.yaml")

    # IPM parameters
    width_m = 20
    height_m = 40
    resolution = 20
    cc = True
    batch = True
    toDrone = True

    if toDrone:
        with open(os.path.abspath(drone_config_path)) as stream:
            droneConfig = yaml.safe_load(stream)

    cameraConfigs, imagePaths, outputFilenames = load_camera_configs_and_images(
        camera_config_paths, image_dirs, batch=batch
    )

    cams = [Camera(config) for config in cameraConfigs]
    if toDrone:
        drone = Camera(droneConfig)

    if not toDrone:
        pxPerM = (resolution, resolution)
        outputRes = (int(width_m * pxPerM[0]), int(height_m * pxPerM[1]))
    else:
        outputRes = (int(2 * droneConfig["py"]), int(2 * droneConfig["px"]))
        dx = outputRes[1] / droneConfig["fx"] * droneConfig["ZCam"]
        dy = outputRes[0] / droneConfig["fy"] * droneConfig["ZCam"]
        pxPerM = (outputRes[0] / dy, outputRes[1] / dx)

    shift = (outputRes[0] / 2.0, outputRes[1] / 2.0)
    if toDrone:
        shift = (
            shift[0] + droneConfig["YCam"] * pxPerM[0],
            shift[1] - droneConfig["XCam"] * pxPerM[1]
        )

    M = np.array([
        [1.0 / pxPerM[1], 0.0, -shift[1] / pxPerM[1]],
        [0.0, -1.0 / pxPerM[0], shift[0] / pxPerM[0]],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ])

    IPMs = [np.linalg.inv(cam.P.dot(M)) for cam in cams]
    masks = compute_masks(cameraConfigs, outputRes, droneConfig, pxPerM)

    interpMode = cv2.INTER_NEAREST if cc else cv2.INTER_LINEAR

    filename = "test"
    hsy = 1
    while not rospy.is_shutdown():
        images = node.images
        if any(image is None for image in images):
            print("Waiting for images...", hsy)
            hsy += 1
            rospy.sleep(0.1) # 10 Hz
            continue
        warpedImages = [cv2.warpPerspective(img, IPM, (outputRes[1], outputRes[0]), flags=interpMode)
                        for img, IPM in zip(images, IPMs)]

        # Apply masks
        for warpedImg, mask in zip(warpedImages, masks):
            warpedImg[mask] = 0

        # Stitch images
        birdsEyeView = np.zeros(warpedImages[0].shape, dtype=np.uint8)
        for wImg in warpedImages:
            mask = np.any(wImg != (0, 0, 0), axis=-1)
            birdsEyeView[mask] = wImg[mask]
        node.image_pub.publish(node.bridge.cv2_to_imgmsg(birdsEyeView, "bgr8"))
        # cv2.imshow("Bird's Eye View", birdsEyeView)
        # cv2.waitKey(1)

        if args.show:
            print("Showing images...")
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            font_color = (255, 255, 255)
            thickness = 2
            line_type = cv2.LINE_AA

            base_shape = images[0].shape if len(images) > 0 else (480, 640, 3)
            blank_image = np.zeros(base_shape, dtype=np.uint8)

            # Extract or create placeholders for each camera image
            cam_names = ["front", "rear", "left", "right"]
            labeled_cams = []
            for i in range(4):
                cam_img = images[i] if i < len(images) else blank_image.copy()
                cv2.putText(cam_img, cam_names[i], (10,30), font, font_scale, font_color, thickness, line_type)
                labeled_cams.append(cam_img)

            # Create a 2x2 grid of camera images
            top_row = cv2.hconcat([labeled_cams[0], labeled_cams[1]])
            bottom_row = cv2.hconcat([labeled_cams[2], labeled_cams[3]]) if len(labeled_cams) > 2 else blank_image
            camera_grid = cv2.vconcat([top_row, bottom_row])

            bev_resized = birdsEyeView
            # print("bev shape1:", bev_resized.shape) # 604, 964
            if camera_grid.shape[1] != bev_resized.shape[1]:
                bev_resized = cv2.resize(bev_resized, (camera_grid.shape[1], bev_resized.shape[0]))
            # print("bev shape2:", bev_resized.shape) # 604, 1928

            cv2.putText(bev_resized, "Bird's Eye View", (10,30), font, font_scale, font_color, thickness, line_type)

            # Combine camera grid and BEV
            display_image = cv2.vconcat([camera_grid, bev_resized])

            scale_factor = 0.5
            new_width = int(display_image.shape[1] * scale_factor)
            new_height = int(display_image.shape[0] * scale_factor)
            display_image = cv2.resize(display_image, (new_width, new_height), interpolation=cv2.INTER_AREA)

            cv2.namedWindow(filename, cv2.WINDOW_NORMAL)
            cv2.imshow(filename, display_image)
            
            key = cv2.waitKey(1) & 0xFF  # Get the last 8 bits of the keypress
            if key == ord('s'):  # If 's' key is pressed
                output_dir = os.path.join(current_dir, "output")
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                save_path = os.path.join(output_dir, "bev_image.png")
                cv2.imwrite(save_path, birdsEyeView)
                print(f"Bird's Eye View image saved to {save_path}")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()