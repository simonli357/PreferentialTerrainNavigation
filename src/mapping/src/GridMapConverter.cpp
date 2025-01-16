#include <iostream>
#include <opencv2/opencv.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include "GridMapConverter.hpp"

int main(int argc, char** argv)
{
    // Load your color-segmented image
    cv::Mat colorImage = cv::imread("/home/slsecret/PreferentialTerrainNavigation/src/mapping/data/bev_image.png");
    if (colorImage.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }

    // Create our GridMapConverter. 
    // resolution_ = 1.0 means 1 pixel = 1 meter (for example).
    GridMapConverter gmConverter("cost", 0.5f);

    // Convert the color image to a grid map
    grid_map::GridMap costMap = gmConverter.convertToGridMap(colorImage);

    // Visualize the "cost" layer (as an 8-bit grayscale image).
    cv::Mat costImageVis = gmConverter.visualizeGridMapLayer(costMap, "cost", 0.0f, 255.0f);

    // Show in OpenCV windows
    cv::imshow("Cost GridMap (Grayscale)", costImageVis);
    cv::waitKey(0);

    return 0;
}
