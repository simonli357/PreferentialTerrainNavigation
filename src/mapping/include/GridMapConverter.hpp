#ifndef GRID_MAP_CONVERTER_HPP
#define GRID_MAP_CONVERTER_HPP

#include <opencv2/opencv.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <string>
#include <stdexcept>

#include "ImageToCost.hpp"

class GridMapConverter
{
public:
  GridMapConverter(const std::string& layerName = "cost",
                   float resolution = 1.0f)
    : layerName_(layerName)
    , resolution_(resolution)
  {}

  /**
   * @brief Convert a color segmented image (Cityscapes) to a GridMap.
   * The resulting grid map has one layer (layerName_) with float cost data.
   */
  grid_map::GridMap convertToGridMap(const cv::Mat& colorImage)
  {
    if (colorImage.empty() || colorImage.channels() != 3) {
      throw std::invalid_argument("Input color image must be a 3-channel image.");
    }

    // 1. Convert color to cost (grayscale)
    ImageToCost converter;
    cv::Mat grayImage = converter.convert(colorImage);

    // 2. Create a grid map with one layer (e.g., "cost").
    grid_map::GridMap gridMap({layerName_});
    gridMap.setFrameId("map");

    // 3. Set the geometry. 
    //    - width  = number_of_columns * resolution
    //    - height = number_of_rows    * resolution
    //    - The center is at (0,0) for demonstration, but you can shift as needed.
    double width = static_cast<double>(grayImage.cols) * resolution_;
    double height = static_cast<double>(grayImage.rows) * resolution_;
    gridMap.setGeometry(grid_map::Length(width, height), resolution_, grid_map::Position(0.0, 0.0));

    // 4. Fill in the cost data.
    // NOTE: grid_map uses (row, col) indexing internally.
    grid_map::Matrix& data = gridMap[layerName_];
    for (int row = 0; row < grayImage.rows; ++row) {
      for (int col = 0; col < grayImage.cols; ++col) {
        // Cast uchar [0..255] -> float
        float costValue = static_cast<float>(grayImage.at<uchar>(row, col));
        data(row, col) = costValue;
      }
    }

    return gridMap;
  }

  /**
   * @brief Visualize the specified layer of a GridMap as an OpenCV image.
   * @param gridMap  The grid map to visualize.
   * @param layer    The layer in the grid map to convert to CV image (e.g. "cost").
   * @param minValue The minimum possible value in that layer.
   * @param maxValue The maximum possible value in that layer.
   * @return OpenCV mat for visualization (CV_8UC1).
   */
  cv::Mat visualizeGridMapLayer(const grid_map::GridMap& gridMap,
                                const std::string& layer = "cost",
                                float minValue = 0.0f, 
                                float maxValue = 255.0f) const
  {
    // Convert from grid map to an 8-bit image for visualization
    cv::Mat cvImage;
    grid_map::GridMapCvConverter::toImage<float, 1>(
      gridMap, layer, CV_8UC1, minValue, maxValue, cvImage);

    return cvImage;
  }

private:
  std::string layerName_;
  float resolution_;
};

#endif // GRID_MAP_CONVERTER_HPP
