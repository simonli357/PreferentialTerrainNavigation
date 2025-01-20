#include "Planner.hpp"

#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <geometry_msgs/PoseStamped.h>

Planner::Planner(const grid_map::GridMap& gridMap, const std::string& layerName)
  : gridMap_(gridMap)
  , layerName_(layerName)
{
  // Base class constructor simply stores the map and the layer name.
}

nav_msgs::Path Planner::toPathMsg(const std::vector<grid_map::Index>& pathIndices,
                                  const std::string& frameId) const
{
  nav_msgs::Path pathMsg;
  pathMsg.header.stamp = ros::Time::now();
  pathMsg.header.frame_id = frameId;

  // Convert each grid_map::Index to a world position and then to a PoseStamped
  for (const auto& idx : pathIndices) {
    grid_map::Position pos;
    if (!gridMap_.getPosition(idx, pos)) {
      // If for some reason the index is invalid, skip.
      continue;
    }

    geometry_msgs::PoseStamped pose;
    pose.header = pathMsg.header;
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();
    // Neutral orientation
    pose.pose.orientation.w = 1.0;

    pathMsg.poses.push_back(pose);
  }

  return pathMsg;
}

grid_map_msgs::GridMap Planner::toGridMapMsg() const
{
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap_, msg);
  msg.info.header.stamp = ros::Time::now();
  // Optionally set frame_id if needed, e.g. msg.info.header.frame_id = "map";
  return msg;
}

