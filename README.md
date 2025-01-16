# Preferential Terrain Navigation

## Overview
Preferential Terrain Navigation is a research project focused on developing an autonomous navigation system for mobile robots capable of traversing unstructured, off-road environments. Unlike many existing navigation systems that prioritize the shortest path, this system seeks to emulate human-like decision-making by considering terrain preferences. For example, a human would typically choose a sidewalk over a snowy path, even if the sidewalk is longer.

The system integrates terrain recognition with optimal control to achieve this goal. The approach consists of two key steps:

1. **Terrain Recognition and Mapping**: Using computer vision to analyze the environment and generate a terrain map that incorporates human-like terrain preferences.
2. **Path Planning and Refinement**: Using the terrain map to plan a global path through trajectory optimization and refining the route using model predictive control (MPC).

This repository contains the implementation of the system using the CARLA simulator and the Robot Operating System (ROS).

---

## Features
- **Terrain Recognition**: Leveraging computer vision and segmentation techniques to classify terrain types and create a preference-based terrain map.
- **Path Planning**: Utilizing trajectory optimization to compute global paths that align with human-like terrain preferences.
- **Path Refinement**: Applying model predictive control (MPC) to adjust the planned route dynamically in response to real-time conditions.
- **Simulation Environment**: Using CARLA to simulate unstructured environments and test navigation algorithms.

---

## Prerequisites

### **CARLA Simulator**: 
[Download CARLA](https://carla.org/) version 0.9.13.
### **Robot Operating System (ROS)**: 
[Install ROS](http://wiki.ros.org/ROS/Installation) (tested with ROS Noetic).
### **Eigen**:
- Installation:
```bash
sudo apt install libeigen3-dev
```
### **Python**: 
Python 3.8 or higher.
### **Grid_Map Library**:
- Installation:
```bash
sudo apt install ros-<your-ros-distro>-grid-map*
```
### Additional dependencies are managed through the included `CMakeLists.txt` and ROS packages.

---

## Getting Started

### Useful Commands

#### Start CARLA Simulator
```bash
cd ~/CARLA_0.9.13
./CarlaUE4.sh -prefernvidia
```

#### Start ROS Bridge
```bash
roslaunch carla_ros_bridge carla_ros_bridge.launch
roslaunch carla_spawn_objects carla_spawn_objects.launch
roslaunch carla_manual_control carla_manual_control.launch
# or equivalently
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```

#### Start Waypoint Publisher and AD Demo
```bash
roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch
```

#### Start AD Demo with Scenario
```bash
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch
```

#### Start RViz
```xml
<node pkg="rviz" type="rviz" name="rviz" args="-d $(find carla_ad_demo)/config/carla_ad_demo.rviz" required="true" output="screen">
    <remap from="carla/ego_vehicle/spectator_pose" to="/carla/ego_vehicle/rgb_view/control/set_transform"/>
</node>
```

#### Start Segmentation Node
```bash
rosrun segmentation segmentation_node.py
```

#### Start Bird’s Eye View (BEV) Node
```bash
rosrun bev ipm_node.py
```

---

### Useful Topics

#### Sensor Topics
- **IMU**: `/carla/ego_vehicle/imu` (Message Type: `sensor_msgs/Imu`)
- **GNSS**: `/carla/ego_vehicle/gnss` (Message Type: `sensor_msgs/NavSatFix`)
- **Odometry**: `/carla/ego_vehicle/odometry` (Message Type: `nav_msgs/Odometry`)
- **Front RGB Camera**: `/carla/ego_vehicle/camera/rgb/front/image_color` (Message Type: `sensor_msgs/Image`)
- **Speedometer**: `/carla/ego_vehicle/speedometer` (Message Type: `std_msgs/Float64`)
- **Front Radar**: `/carla/ego_vehicle/radar_front` (Message Type: `carla_msgs/CarlaRadar`)
- **LiDAR**: `/carla/ego_vehicle/lidar` (Message Type: `sensor_msgs/PointCloud2`)
- **Laser Scan**: `/scan` (Message Type: `sensor_msgs/LaserScan`) – added using the `pointcloud_to_laserscan` package.

---

## Repository Structure
```
PreferentialTerrainNavigation
├── src
│   ├── bev                  # Bird’s Eye View (BEV) generation
│   ├── segmentation         # Terrain segmentation
│   ├── mapping              # Grid Map Creation from Segmented BEV Image
│   ├── planning             # Path planning and MPC modules
│   ├── ros-bridge           # Integration with ROS and CARLA
│   ├── CMakeLists.txt       # Build configuration
```

---

## How to Contribute
1. Fork the repository and create a new branch for your feature or bug fix.
2. Ensure your code adheres to the existing style and structure.
3. Submit a pull request with a detailed description of your changes.

---

## Acknowledgments
- [CARLA Simulator](https://carla.org/)
- [ROS](http://wiki.ros.org/)
- The research community exploring autonomous navigation in unstructured environments.

---

## Contact
For questions or collaboration, please contact Simon Li at xi.yang.li@mail.mcgill.ca.

