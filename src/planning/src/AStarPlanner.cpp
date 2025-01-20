#include <ros/ros.h>
#include "AStarPlanner.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <iostream>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <geometry_msgs/PoseStamped.h>

AStarPlanner::AStarPlanner(const grid_map::GridMap& gridMap,
                           const std::string& layerName)
  : gridMap_(gridMap),
    layerName_(layerName)
{
  // Constructor simply copies references into internal members.
}

std::vector<grid_map::Index> AStarPlanner::plan(const grid_map::Index& startIndex,
                                                const grid_map::Index& goalIndex)
{
  // Priority queue (open set) for A*, sorted by fCost (lowest first).
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openSet;
  
  // Keep track of visited (closed set).
  std::unordered_map<size_t, bool> visited;
  
  // gScore: map from indexKey -> cost so far.
  std::unordered_map<size_t, double> gScore;
  
  // cameFrom: map from indexKey -> parent index
  std::unordered_map<size_t, grid_map::Index> cameFrom;

  // Initialize start node.
  AStarNode startNode;
  startNode.index = startIndex;
  startNode.gCost = 0.0;
  startNode.fCost = heuristic(startIndex, goalIndex);

  openSet.push(startNode);
  gScore[indexToKey(startIndex)] = 0.0;

  // A* main loop.
  while (!openSet.empty()) {
    AStarNode current = openSet.top();
    openSet.pop();

    // If we already processed this node (visited), skip.
    size_t currentKey = indexToKey(current.index);
    if (visited[currentKey]) {
      continue;
    }
    visited[currentKey] = true;

    // Check if we have reached the goal.
    if (current.index(0) == goalIndex(0) && current.index(1) == goalIndex(1)) {
      return reconstructPath(cameFrom, goalIndex);
    }

    // Explore neighbors.
    for (const auto& dir : directions_) {
      grid_map::Index neighbor(current.index(0) + dir.first,
                               current.index(1) + dir.second);

      // Check if inside bounds.
      if (!isInBounds(neighbor)) {
        continue;
      }

      size_t neighborKey = indexToKey(neighbor);

      // If neighbor was visited, skip.
      if (visited[neighborKey]) {
        continue;
      }

      // Get the cell cost from the GridMap layer.
      float cost = gridMap_.at(layerName_, neighbor);

      // Optional: treat cost >= 100 as obstacles (untraversable).
      // if (cost >= 100.0f) {
      //   continue;
      // }

      // If you allow high cost traversal, just add it to the path cost:
      double tentative_gCost = current.gCost + cost;

      // If we haven't seen this neighbor yet, or we found a cheaper path:
      if (gScore.find(neighborKey) == gScore.end() ||
          tentative_gCost < gScore[neighborKey]) {
        gScore[neighborKey] = tentative_gCost;
        cameFrom[neighborKey] = current.index;

        double hCost = heuristic(neighbor, goalIndex);
        double fCost = tentative_gCost + hCost;

        AStarNode neighborNode;
        neighborNode.index = neighbor;
        neighborNode.gCost = tentative_gCost;
        neighborNode.fCost = fCost;
        
        openSet.push(neighborNode);
      }
    }
  }

  // If we exit the loop, no path was found.
  std::cerr << "[AStarPlanner] No path found from start to goal!" << std::endl;
  return {};
}

nav_msgs::Path AStarPlanner::toPathMsg(const std::vector<grid_map::Index>& pathIndices,
                                       const std::string& frameId) const
{
  nav_msgs::Path pathMsg;
  pathMsg.header.stamp = ros::Time::now();
  pathMsg.header.frame_id = frameId;

  // For each grid_map::Index, convert to world coordinate (x,y) and build a PoseStamped.
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
    // Keep orientation neutral (w=1), or define if needed
    pose.pose.orientation.w = 1.0;

    pathMsg.poses.push_back(pose);
  }

  return pathMsg;
}

grid_map_msgs::GridMap AStarPlanner::toGridMapMsg() const
{
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap_, msg);
  msg.info.header.stamp = ros::Time::now();
  // Also set frame_id if needed
  // msg.info.header.frame_id = "map"; 
  return msg;
}

double AStarPlanner::heuristic(const grid_map::Index& a, const grid_map::Index& b) const
{
  // Euclidean distance
  double dr = static_cast<double>(a(0) - b(0));
  double dc = static_cast<double>(a(1) - b(1));
  return std::sqrt(dr * dr + dc * dc);
}

bool AStarPlanner::isInBounds(const grid_map::Index& index) const
{
  // Ensure row & col are within the grid dimensions.
  return (index(0) >= 0 && index(0) < gridMap_.getSize()(0) &&
          index(1) >= 0 && index(1) < gridMap_.getSize()(1));
}

std::vector<grid_map::Index> AStarPlanner::reconstructPath(
    const std::unordered_map<size_t, grid_map::Index>& cameFrom,
    const grid_map::Index& current) const
{
  std::vector<grid_map::Index> path;
  path.push_back(current);

  grid_map::Index cur = current;
  while (true) {
    size_t curKey = indexToKey(cur);
    auto it = cameFrom.find(curKey);
    if (it == cameFrom.end()) {
      break;
    }
    cur = it->second;
    path.push_back(cur);
  }

  std::reverse(path.begin(), path.end());
  return path;
}

size_t AStarPlanner::indexToKey(const grid_map::Index& idx) const
{
  // Simple approach: combine row & col into a single integer
  // Warning: watch out for potential overflow if map is huge.
  // Adjust bit shifts or use a better hashing for large maps.
  return (static_cast<size_t>(idx(0)) << 16) ^ static_cast<size_t>(idx(1));
}

// int main()
// {
//   // Create a GridMap with one layer "cost"
//   grid_map::GridMap map({"cost"});
//   // Set geometry: for example, a 5m x 5m map with 0.1m resolution (50x50 cells).
//   map.setGeometry(grid_map::Length(5.0, 5.0), 0.1);

//   // Fill with some cost values. here we make a star pattern.
//     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//         const grid_map::Index index(*it);
//         float cost = 1.0f;
//         if (index(0) == 25 || index(1) == 25) {
//         cost = 100.0f;
//         }
//         map.at("cost", index) = cost;
//     }
  

//   // Create an AStarPlanner
//   AStarPlanner planner(map, "cost");

//   // Define start and goal in index space (top-left to bottom-right).
//   grid_map::Index startIndex(0, 0);
//   grid_map::Index goalIndex(49, 49);

//   // Get the path
//   auto path = planner.plan(startIndex, goalIndex);

//   if (!path.empty()) {
//     std::cout << "Path found! Size: " << path.size() << std::endl;
//     for (auto& idx : path) {
//       std::cout << "(" << idx(0) << ", " << idx(1) << ") -> ";
//     }
//     std::cout << "GOAL\n";
//   } else {
//     std::cout << "No path found.\n";
//   }

//   return 0;
// }
