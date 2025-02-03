#include "AStarPlanner.hpp"
#include <ros/ros.h>
#include <limits>
#include <chrono>

constexpr double AStarPlanner::INF;

std::vector<grid_map::Index> AStarPlanner::plan(const grid_map::GridMap& gridMap,
                                                const std::string& layerName,
                                                const grid_map::Index& startIndex,
                                                const grid_map::Index& goalIndex)
{
  auto start = std::chrono::high_resolution_clock::now();
  // Fetch grid dimensions.
  const auto& mapSize = gridMap.getSize();
  int rows = mapSize(0);
  int cols = mapSize(1);

  // Prepare / reset all internal data for new search.
  resetData(rows, cols);

  // Clear the priority queue (open set).
  openSet_ = std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>();

  // Initialize the start node and push to open set.
  const int startId = to1D(startIndex(0), startIndex(1));
  AStarNode startNode;
  startNode.index = startIndex;
  startNode.gCost = 0.0;
  startNode.fCost = heuristic(startIndex, goalIndex);
  openSet_.push(startNode);

  gScore_[startId] = 0.0;
  visited_[startId] = false;

  // A* main loop.
  while (!openSet_.empty())
  {
    AStarNode current = openSet_.top();
    openSet_.pop();

    const auto& curIdx = current.index;
    const int curId = to1D(curIdx(0), curIdx(1));

    // If this node is already visited or we found a better path, skip it.
    if (visited_[curId]) {
      continue;
    }
    if (current.gCost > gScore_[curId]) {
      // This is an outdated entry in the priority queue.
      continue;
    }

    // Mark current as visited.
    visited_[curId] = true;

    // Check if goal reached.
    if (current.index(0) == goalIndex(0) && current.index(1) == goalIndex(1)) {
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      ROS_INFO_STREAM("[AStarPlanner] Path found in " << elapsed.count() << " seconds.");
      return reconstructPath(goalIndex);
    }

    // Explore neighbors.
    for (const auto& dir : directions_) {
      grid_map::Index neighbor(curIdx(0) + dir.first,
                               curIdx(1) + dir.second);

      // Check if inside bounds.
      if (!isInBounds(neighbor)) {
        continue;
      }
      const int neighborId = to1D(neighbor(0), neighbor(1));

      // If already visited, skip.
      if (visited_[neighborId]) {
        continue;
      }

      // Get the cell cost from the GridMap layer.
      float cellCost = gridMap.at(layerName, neighbor);

      // Treat high cost (or special value) as an obstacle if desired.
      // e.g. if cellCost >= 200, skip. Otherwise incorporate it into gCost.
      if (cellCost >= 200.0f) {
        continue;
      }

      // Tentative gCost = current cost + cell cost (or +1 if uniform cost).
      double tentativeG = gScore_[curId] + static_cast<double>(cellCost);

      // If we found a better path to the neighbor:
      if (tentativeG < gScore_[neighborId]) {
        gScore_[neighborId] = tentativeG;
        cameFrom_[neighborId] = curIdx;

        AStarNode neighborNode;
        neighborNode.index = neighbor;
        neighborNode.gCost = tentativeG;
        neighborNode.fCost = tentativeG + heuristic(neighbor, goalIndex);

        openSet_.push(neighborNode);
      }
    }
  }

  // If we exit the loop, no path was found.
  ROS_WARN_STREAM("[AStarPlanner] No path found from start to goal!");
  return {};
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
  // Ensure row & col are within [0, nRows_) and [0, nCols_).
  return (index(0) >= 0 && index(0) < nRows_ &&
          index(1) >= 0 && index(1) < nCols_);
}

std::vector<grid_map::Index> AStarPlanner::reconstructPath(const grid_map::Index& goalIndex) const
{
  std::vector<grid_map::Index> path;
  grid_map::Index current = goalIndex;

  while (true)
  {
    path.push_back(current);
    const int curId = to1D(current(0), current(1));

    // If no parent, we've reached the start node.
    // We store parents in cameFrom_, so if it wasn't set, it's default.
    if (cameFrom_[curId](0) < 0) {
      break;
    }
    current = cameFrom_[curId];
  }

  std::reverse(path.begin(), path.end());
  return path;
}

void AStarPlanner::resetData(int rows, int cols)
{
  // If the grid size has changed, reallocate internal vectors.
  // Otherwise, just reset them.
  if (rows != nRows_ || cols != nCols_) {
    nRows_ = rows;
    nCols_ = cols;

    visited_.resize(rows * cols, false);
    gScore_.resize(rows * cols, INF);
    cameFrom_.resize(rows * cols, grid_map::Index(-1, -1));
  }
  else {
    std::fill(visited_.begin(), visited_.end(), false);
    std::fill(gScore_.begin(), gScore_.end(), INF);
    std::fill(cameFrom_.begin(), cameFrom_.end(), grid_map::Index(-1, -1));
  }
}
