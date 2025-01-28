#include "Planner.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

std::vector<grid_map::Index> Planner::planFromPosition(const grid_map::GridMap& gridMap,
                                                       const std::string& layerName,
                                                       const grid_map::Position& startPos,
                                                       const grid_map::Position& goalPos)
{
  grid_map::Index startIndex, goalIndex;

  // Convert world positions to grid indices
  bool gotStart = gridMap.getIndex(startPos, startIndex);
  bool gotGoal  = gridMap.getIndex(goalPos, goalIndex);

  if (!gotStart) {
    ROS_WARN("Planner::planFromPosition() - Could not convert startPos to an index!");
    return {};
  }
  if (!gotGoal) {
    ROS_WARN("Planner::planFromPosition() - Could not convert goalPos to an index!");
    return {};
  }

  return plan(gridMap, layerName, startIndex, goalIndex);
}

nav_msgs::Path Planner::toPathMsg(const std::vector<grid_map::Index>& pathIndices,
                                  const grid_map::GridMap& gridMap,
                                  const std::string& frameId) const
{
  nav_msgs::Path pathMsg;
  pathMsg.header.stamp = ros::Time::now();
  pathMsg.header.frame_id = frameId;

  for (const auto& idx : pathIndices) {
    grid_map::Position pos;
    // Convert from grid cell index to world coordinates
    if (!gridMap.getPosition(idx, pos)) {
      // If for some reason the index is invalid, skip
      continue;
    }

    geometry_msgs::PoseStamped pose;
    pose.header = pathMsg.header;
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();
    // Keep orientation neutral
    pose.pose.orientation.w = 1.0;

    pathMsg.poses.push_back(pose);
  }

  return pathMsg;
}

grid_map_msgs::GridMap Planner::toGridMapMsg(const grid_map::GridMap& gridMap) const
{
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, msg);
  msg.info.header.stamp = ros::Time::now();
  msg.info.header.frame_id = "map";
  return msg;
}
