#pragma once

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>
#include <string>
#include "Constants.h"

using namespace Constants;
class ImageToCost {
public:
    ImageToCost() {
        initializeColorMap();
    }

    cv::Mat convert(const cv::Mat& colorImage) {
        if (colorImage.empty() || colorImage.channels() != 3) {
            throw std::invalid_argument("Input image must be a non-empty 3-channel image.");
        }

        cv::Mat grayImage(colorImage.rows, colorImage.cols, CV_8UC1);

        for (int y = 0; y < colorImage.rows; ++y) {
            for (int x = 0; x < colorImage.cols; ++x) {
                cv::Vec3b color = colorImage.at<cv::Vec3b>(y, x);
                grayImage.at<uchar>(y, x) = getCost(color);
            }
        }

        return grayImage;
    }

    cv::Mat convertToColor(const cv::Mat& costImage) {
        if (costImage.empty() || costImage.channels() != 1) {
            throw std::invalid_argument("Input image must be a non-empty single-channel image.");
        }

        cv::Mat colorImage(costImage.rows, costImage.cols, CV_8UC3);

        for (int y = 0; y < costImage.rows; ++y) {
            for (int x = 0; x < costImage.cols; ++x) {
                uchar cost = costImage.at<uchar>(y, x);
                colorImage.at<cv::Vec3b>(y, x) = getColor(cost);
            }
        }

        return colorImage;
    }

private:
    struct Vec3bHash {
        size_t operator()(const cv::Vec3b& color) const {
            return std::hash<int>()(color[0]) ^ (std::hash<int>()(color[1]) << 1) ^ (std::hash<int>()(color[2]) << 2);
        }
    };

    static std::vector<std::string> CLASSES;
    std::unordered_map<cv::Vec3b, float, Vec3bHash> colorCostMap;
    std::unordered_map<float, cv::Vec3b> costColorMap;

    void initializeColorMap() {
        for (size_t i = 0; i < COLOR_MAP.size(); ++i) {
            colorCostMap[COLOR_MAP[i]] = TERRAIN_COSTS[i];
            costColorMap[TERRAIN_COSTS[i]] = COLOR_MAP[i];
        }
    }

    uchar getCost(const cv::Vec3b& color) const {
        auto it = colorCostMap.find(color);
        if (it != colorCostMap.end()) {
            return static_cast<uchar>(it->second);
        }
        return 0; // Default cost for unmatched colors
    }

    cv::Vec3b getColor(float cost) const {
        auto it = costColorMap.find(cost);
        if (it != costColorMap.end()) {
            return it->second;
        }
        return {0, 0, 0}; // Default color for unmatched costs
    }
};
