cmake_minimum_required(VERSION 3.0.2)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_cv
  grid_map_costmap_2d
  grid_map_sdf
  grid_map_pcl
  grid_map_octomap
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp cv_bridge sensor_msgs geometry_msgs std_msgs grid_map_core grid_map_ros grid_map_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(TerrainMap src/TerrainMapOld.cpp)
target_link_libraries(TerrainMap ${catkin_LIBRARIES})

add_executable(TerrainMap2 src/TerrainMap2.cpp)
target_link_libraries(TerrainMap2 ${catkin_LIBRARIES})

add_executable(image_to_gridmap_demo src/ImageToGridmapDemo.cpp)
target_link_libraries(image_to_gridmap_demo ${catkin_LIBRARIES})

add_executable(ImageToCost src/ImageToCost.cpp)
target_link_libraries(ImageToCost ${catkin_LIBRARIES})

add_executable(main 
  src/TerrainMap.cpp
  src/main.cpp
)
target_link_libraries(main ${catkin_LIBRARIES})

# add_executable(GridMapConverter src/GridMapConverter.cpp)
# target_link_libraries(GridMapConverter ${catkin_LIBRARIES})
