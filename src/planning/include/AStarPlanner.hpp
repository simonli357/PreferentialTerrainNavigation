#pragma once

#include "Planner.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

/**
 * @brief Class that implements an A* path planner for a single-layer GridMap.
 *        Inherits from Planner.
 */
class AStarPlanner : public Planner
{
public:
  /**
   * @brief By design, no constructor arguments for the map or layer name,
   *        because we receive those at plan-time. We could still allow
   *        config parameters, if needed (e.g. diagonal movement, cost thresholds).
   */
  AStarPlanner() = default;
  virtual ~AStarPlanner() = default;

  /**
   * @brief Finds a path from start to goal using A* search.
   * @param gridMap    The GridMap on which to plan.
   * @param startIndex The start cell index (row, col) in the grid.
   * @param goalIndex  The goal cell index (row, col) in the grid.
   * @return A vector of grid_map::Index forming the path from start to goal.
   *         If no path is found, returns an empty vector.
   */
  std::vector<grid_map::Index> plan(const grid_map::GridMap& gridMap,
                                    const std::string& layerName,
                                    const grid_map::Index& startIndex,
                                    const grid_map::Index& goalIndex) override;

private:
  /// Internal struct representing a node in the A* open set.
  struct AStarNode
  {
    grid_map::Index index;
    double gCost;  ///< Cost from the start node
    double fCost;  ///< gCost + heuristic
  };

  /// Comparison functor for the priority queue (min-heap by fCost).
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
  bool isInBounds(const grid_map::Index& index, const grid_map::GridMap& gridMap) const;

  /// Reconstructs the path once the goal is reached.
  std::vector<grid_map::Index> reconstructPath(
      const std::unordered_map<size_t, grid_map::Index>& cameFrom,
      const grid_map::Index& current) const;

  /// Converts a grid_map::Index to a unique key for hashing in std::unordered_map.
  size_t indexToKey(const grid_map::Index& idx) const;

  // Movement directions (8-connected). Remove diagonals for 4-connected.
  const std::vector<std::pair<int, int>> directions_ = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };
};

