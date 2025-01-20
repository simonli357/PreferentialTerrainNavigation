#pragma once

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>

#include <mapping/Constants.h>
#include <mapping/ImageToCost.hpp>
#include <mapping/TerrainMap.hpp>

#include <nav_msgs/Path.h>
#include <grid_map_msgs/GridMap.h>

/**
 * @brief Class that implements an A* path planner for a single-layer GridMap.
 */
class AStarPlanner
{
public:
  /**
   * @brief Constructor
   * @param gridMap   The GridMap on which to plan.
   * @param layerName The name of the cost layer (e.g., "cost").
   */
  AStarPlanner(const grid_map::GridMap& gridMap, const std::string& layerName);

  /**
   * @brief Finds a path from start to goal using A* search.
   * @param startIndex The start cell index (row, col) in the grid.
   * @param goalIndex  The goal cell index (row, col) in the grid.
   * @return A vector of grid_map::Index forming the path from start to goal.
   *         If no path is found, returns an empty vector.
   */
  std::vector<grid_map::Index> plan(const grid_map::Index& startIndex,
                                    const grid_map::Index& goalIndex);

  /**
   * @brief Convert a sequence of grid_map::Index (the A* path) to a nav_msgs::Path.
   * @param pathIndices   The A* path as a vector of grid_map::Index.
   * @param frameId       The frame to use for the Path header (default "map").
   * @return              A nav_msgs::Path message with each pose in map coordinates.
   */
  nav_msgs::Path toPathMsg(const std::vector<grid_map::Index>& pathIndices,
                           const std::string& frameId = "map") const;

  /**
   * @brief Convert the internal grid_map::GridMap to a grid_map_msgs::GridMap message.
   * @return The GridMap in message form, ready to be published.
   */
  grid_map_msgs::GridMap toGridMapMsg() const;
  
private:
  /// Internal struct representing a node in the A* open set.
  struct AStarNode
  {
    grid_map::Index index;
    double gCost;  ///< Cost from the start node
    double fCost;  ///< gCost + heuristic
  };

  /// Comparison functor for priority queue (min-heap by fCost).
  struct CompareF
  {
    bool operator()(const AStarNode& a, const AStarNode& b) const
    {
      return a.fCost > b.fCost;
    }
  };

  /// Heuristic function (Euclidean distance).
  double heuristic(const grid_map::Index& a, const grid_map::Index& b) const;

  /// Checks if a given cell index is within the bounds of the stored grid map.
  bool isInBounds(const grid_map::Index& index) const;

  /// Reconstructs the path once the goal is reached.
  std::vector<grid_map::Index> reconstructPath(
      const std::unordered_map<size_t, grid_map::Index>& cameFrom,
      const grid_map::Index& current) const;

  /// Converts a grid_map::Index to a unique key for hashing in std::unordered_map.
  size_t indexToKey(const grid_map::Index& idx) const;

private:
  grid_map::GridMap gridMap_;  ///< A copy of the input GridMap
  std::string layerName_;      ///< Name of the layer containing cost data

  // Movement directions (8-connected). Remove diagonals for 4-connected.
  const std::vector<std::pair<int, int>> directions_ = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };
};
