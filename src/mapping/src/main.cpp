#include <ros/ros.h>
#include "mapping/TerrainMap.hpp"
#include "mapping/Constants.h"
#include "mapping/ImageToCost.hpp"
#include <grid_map_cv/grid_map_cv.hpp>

int main(int argc, char** argv)
{
    // initialize ROS
    // ros::init(argc, argv, "grid_map_example");
    // ros::NodeHandle nh;

    // 1) Create global map container
    TerrainMap myGlobalMap;

    // 2) Build a dummy local cost image (964x604, e.g. circle in the center)
    // cv::Mat localMap(Constants::IMG_HEIGHT_PX, Constants::IMG_WIDTH_PX, CV_8UC1, cv::Scalar(0));
    // cv::circle(localMap, 
    //            cv::Point(localMap.cols/2, localMap.rows/2), 
    //            100, 
    //            cv::Scalar(255),
    //            -1);

    cv::Mat colorImage = cv::imread("/home/slsecret/PreferentialTerrainNavigation/src/mapping/data/bev_image.png");
    if (colorImage.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }
    ImageToCost converter;
    cv::Mat localMap = converter.convert(colorImage);

    // 3) The robot's global pose
    double robotX = 50.0;     // center of global map
    double robotY = 70.0;     
    double robotYaw = M_PI/4; // 45 deg

    // 4) Update the global map with the local cost image
    myGlobalMap.updateGlobalMap(localMap, robotX, robotY, robotYaw);

    // 5) Optionally, retrieve the map and inspect the "terrainCost" layer
    auto& gridMap = myGlobalMap.getMap();
    // For demonstration, let's convert the "terrainCost" layer to a cv::Mat so we can visualize it.
    // cv::Mat globalCostImage(gridMap.getSize().y(), gridMap.getSize().x(), CV_8UC1, cv::Scalar(0));
    // for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    //     const grid_map::Index index(*it);
    //     float costVal = gridMap.at("terrainCost", index);
    //     globalCostImage.at<uint8_t>(index(0), index(1)) = static_cast<uint8_t>(costVal);
    // }

    cv::Mat globalCostImage;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
        gridMap, 
        "terrainCost", 
        CV_16UC1,
        0, 
        255, 
        globalCostImage
    );
    // image is 16-bit, convert to 8-bit for visualization
    cv::Mat globalCostImage8;
    double minVal, maxVal;
    cv::minMaxLoc(globalCostImage, &minVal, &maxVal);
    globalCostImage.convertTo(globalCostImage8, CV_8UC1, 255.0/(maxVal));

    cv::Mat displayMap;
    // rotate so that x points to the right and y points up
    cv::rotate(globalCostImage8, displayMap, cv::ROTATE_90_CLOCKWISE); 

    cv::Mat colorImage2 = converter.convertToColor(displayMap);
    std::cout << "colorImage2 size: " << colorImage2.size() << std::endl;

    cv::namedWindow("Global Cost (grid_map)", cv::WINDOW_NORMAL);
    cv::imshow("Global Cost colored (grid_map)", colorImage2);
    cv::imshow("Global Cost (grid_map)", displayMap);
    cv::waitKey(0);

    return 0;
}
