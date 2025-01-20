#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "AStarPlanner.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map_a_star_publisher");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher mapPub = nh.advertise<grid_map_msgs::GridMap>("my_grid_map", 1);
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("my_a_star_path", 1);

    // 1. Create or load a GridMap
    grid_map::GridMap map({"cost"});
    map.setGeometry(grid_map::Length(10.0, 10.0), 0.1, grid_map::Position(0.0, 0.0));
    std::cout << "Created map with size: " << map.getSize().transpose()
            << " (rows x cols)" << std::endl;

    // 2. Fill with a default low cost (e.g., 1.0).
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        map.at("cost", *it) = 1.0;
    }

    // 3. Define a star-shaped polygon in *world coordinates* (same frame as the map).
    //    For simplicity, place the star near the map center. 
    //    This is a rough 5-point star, defined by 10 vertices.
    grid_map::Polygon starPolygon;
    // By default, the frameId can remain empty or match map.getFrameId() if you're using ROS.
    // starPolygon.setFrameId(map.getFrameId()); // optional if used in ROS

    // Let's define a star with approximate radius ~2.0 around the origin:
    //    Outer points at ~2.0 from center, inner points at ~1.0 from center.
    // We'll go around in a circle. Just an example:
    starPolygon.addVertex(grid_map::Position(0.0,  2.0));   // top
    starPolygon.addVertex(grid_map::Position(0.5,  0.7));
    starPolygon.addVertex(grid_map::Position(2.0,  0.7));
    starPolygon.addVertex(grid_map::Position(0.8, -0.2));
    starPolygon.addVertex(grid_map::Position(1.2, -1.8));  
    starPolygon.addVertex(grid_map::Position(0.0, -0.8));  // bottom center
    starPolygon.addVertex(grid_map::Position(-1.2, -1.8));
    starPolygon.addVertex(grid_map::Position(-0.8, -0.2));
    starPolygon.addVertex(grid_map::Position(-2.0,  0.7));
    starPolygon.addVertex(grid_map::Position(-0.5,  0.7));

    // 4. Use PolygonIterator to set a higher cost in the star region.
    //    For example, set cost = 50.0 inside the star. 
    //    (You could also set 100.0 if you want it to be fully impassable.)

    for (grid_map::PolygonIterator polyIt(map, starPolygon); !polyIt.isPastEnd(); ++polyIt) {
        // *polyIt is a grid_map::Index
        map.at("cost", *polyIt) = 50.0;
    }

    // 5. Create the AStarPlanner and plan from top-left to bottom-right
    //    in *index space* (row, col).
    //    - top-left index would be (0, 0)
    //    - bottom-right index would be (rows-1, cols-1)
    //    NOTE: row = 0 is top edge in grid_map, row = N-1 is bottom edge.
    //          col = 0 is left edge, col = M-1 is right edge.

    AStarPlanner planner(map, "cost");

    // 3. Define start and goal in index space
    grid_map::Index startIndex(0, 0);
    grid_map::Index goalIndex(92, 80);

    // 4. Plan
    auto pathIndices = planner.plan(startIndex, goalIndex);

    // 5. Convert to ROS messages
    auto pathMsg = planner.toPathMsg(pathIndices, "map"); // or any frame you'd like
    auto gridMapMsg = planner.toGridMapMsg();
    
    // Publish in a loop
    ros::Rate rate(1.0); // 1 Hz
    while (ros::ok()) {
        gridMapMsg.info.header.stamp = ros::Time::now();
        pathMsg.header.stamp = ros::Time::now();

        mapPub.publish(gridMapMsg);
        pathPub.publish(pathMsg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
