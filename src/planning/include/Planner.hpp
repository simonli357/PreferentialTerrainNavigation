#pragma once

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <nav_msgs/Path.h>
#include <grid_map_msgs/GridMap.h>

/**
 * @brief Abstract base class for path planners operating on a single-layer GridMap.
 */
class Planner
{
public:
  /**
   * @brief Constructor for the Planner base class.
   * @param gridMap   The GridMap on which to plan.
   * @param layerName The name of the cost layer (e.g., "cost").
   */
  Planner(const grid_map::GridMap& gridMap, const std::string& layerName);

  /**
   * @brief Virtual destructor.
   */
  virtual ~Planner() = default;

  /**
   * @brief Pure virtual method for planning a path from start to goal.
   *        Must be overridden by derived classes (e.g. AStarPlanner, RRTPlanner).
   * @param startIndex The start cell index (row, col) in the grid.
   * @param goalIndex  The goal cell index (row, col) in the grid.
   * @return A vector of grid_map::Index forming the path from start to goal.
   *         If no path is found, returns an empty vector.
   */
  virtual std::vector<grid_map::Index> plan(const grid_map::Index& startIndex,
                                            const grid_map::Index& goalIndex) = 0;

  /**
   * @brief Convert a sequence of grid_map::Index (the planned path) to a nav_msgs::Path.
   *        By default, this uses the internal gridMap_ to convert cell indices to world coords.
   * @param pathIndices   The path as a vector of grid_map::Index.
   * @param frameId       The frame to use for the Path header (default "map").
   * @return              A nav_msgs::Path message with each pose in map coordinates.
   */
  virtual nav_msgs::Path toPathMsg(const std::vector<grid_map::Index>& pathIndices,
                                   const std::string& frameId = "map") const;

  /**
   * @brief Convert the internal grid_map::GridMap to a grid_map_msgs::GridMap message.
   * @return The GridMap in message form, ready to be published.
   */
  virtual grid_map_msgs::GridMap toGridMapMsg() const;

protected:
  /// A copy of the input GridMap (could also be stored as a const ref if desired).
  grid_map::GridMap gridMap_;

  /// Name of the layer containing cost data.
  std::string layerName_;
};

