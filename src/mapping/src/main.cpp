#include <ros/ros.h> // optional if you're using ROS
#include "TerrainMap.hpp"
#include "Constants.h"

int main(int argc, char** argv)
{
    // (Optional) initialize ROS
    // ros::init(argc, argv, "grid_map_example");
    // ros::NodeHandle nh;

    // 1) Create our global map container
    TerrainMap myGlobalMap;

    // 2) Build a dummy local cost image (964x604, e.g. circle in the center)
    cv::Mat localMap(Constants::IMG_HEIGHT_PX, Constants::IMG_WIDTH_PX, CV_8UC1, cv::Scalar(0));
    cv::circle(localMap, 
               cv::Point(localMap.cols/2, localMap.rows/2), 
               100, 
               cv::Scalar(255),
               -1);

    // 3) The robot's global pose
    double robotX = 50.0;     // center of global map
    double robotY = 50.0;     
    double robotYaw = -M_PI/4; // 45 deg

    // 4) Update the global map with the local cost image
    myGlobalMap.updateGlobalMap(localMap, robotX, robotY, robotYaw);

    // 5) Optionally, retrieve the map and inspect the "terrainCost" layer
    auto& gridMap = myGlobalMap.getMap();
    // For demonstration, let's convert the "terrainCost" layer to a cv::Mat so we can visualize it.
    cv::Mat globalCostImage(gridMap.getSize().y(), gridMap.getSize().x(), CV_8UC1, cv::Scalar(0));

    for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        float costVal = gridMap.at("terrainCost", index);
        globalCostImage.at<uint8_t>(index(0), index(1)) = static_cast<uint8_t>(costVal);
    }

    cv::namedWindow("Global Cost (grid_map)", cv::WINDOW_NORMAL);
    cv::imshow("Global Cost (grid_map)", globalCostImage);
    cv::waitKey(0);

    return 0;
}
