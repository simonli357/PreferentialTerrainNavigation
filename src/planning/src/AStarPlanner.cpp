#include "AStarPlanner.hpp"
#include "mapping/Constants.h"
#include <ros/ros.h>

std::vector<grid_map::Index> AStarPlanner::plan(const grid_map::GridMap& gridMap,
                                                const std::string& layerName,
                                                const grid_map::Index& startIndex,
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

    size_t currentKey = indexToKey(current.index);
    // If already processed this node (visited), skip.
    if (visited[currentKey]) {
      continue;
    }
    visited[currentKey] = true;

    // Check if goal reached.
    if (current.index(0) == goalIndex(0) && current.index(1) == goalIndex(1)) {
      return reconstructPath(cameFrom, goalIndex);
    }

    // Explore neighbors.
    for (const auto& dir : directions_) {
      grid_map::Index neighbor(current.index(0) + dir.first,
                               current.index(1) + dir.second);

      // Check if inside bounds.
      if (!isInBounds(neighbor, gridMap)) {
        continue;
      }

      size_t neighborKey = indexToKey(neighbor);

      // If neighbor was visited, skip.
      if (visited[neighborKey]) {
        continue;
      }

      // Get the cell cost from the GridMap layer.
      float cost = gridMap.at(layerName, neighbor);

      // Optional: Treat high cost values as obstacles.
      if (cost >= 200.0f) {
        continue;
      }

      // Or simply add it to path cost if you want to allow high cost traversal.
      double tentative_gCost = current.gCost + cost;

      // If we haven't seen this neighbor yet, or found a cheaper path:
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

double AStarPlanner::heuristic(const grid_map::Index& a, const grid_map::Index& b) const
{
  // Euclidean distance
  double dr = static_cast<double>(a(0) - b(0));
  double dc = static_cast<double>(a(1) - b(1));
  return std::sqrt(dr * dr + dc * dc);
}

bool AStarPlanner::isInBounds(const grid_map::Index& index, 
                              const grid_map::GridMap& gridMap) const
{
  // Ensure row & col are within the grid dimensions.
  return (index(0) >= 0 && index(0) < gridMap.getSize()(0) &&
          index(1) >= 0 && index(1) < gridMap.getSize()(1));
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
  // return (static_cast<size_t>(idx(0)) << 16) ^ static_cast<size_t>(idx(1));
  static int maxRow = static_cast<int>(std::max(Constants::GLOBAL_MAP_HEIGHT, Constants::GLOBAL_MAP_WIDTH) / Constants::METERS_PER_CELL);
  return static_cast<size_t>(idx(0)) * maxRow + static_cast<size_t>(idx(1));
}
