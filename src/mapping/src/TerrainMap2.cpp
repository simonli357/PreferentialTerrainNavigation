#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  // <-- Add this

#include <iostream>
#include <map>
#include <string>
#include <array>
#include <vector>

// ---- Your color map, classes, and class-cost definitions ----
static const std::vector<std::array<int, 3>> COLOR_MAP = {
    {0,0,0}, {128,64,128}, {244,35,232}, {70,70,70},  {102,102,156},
    {190,153,153}, {153,153,153}, {250,170,30}, {220,220,0}, {107,142,35},
    {152,251,152}, {70,130,180}, {220,20,60}, {255,0,0}, {0,0,142},
    {0,0,70}, {0,60,100}, {0,80,100}, {0,0,230}, {119,11,32}
};

static const std::vector<std::string> CLASSES = {
    "unlabeled", "road", "sidewalk", "building", "wall", "fence", "pole",
    "traffic light", "traffic sign", "vegetation", "terrain", "sky", "person",
    "rider", "car", "truck", "bus", "train", "motorcycle", "bicycle"
};

static const std::map<std::string, float> CLASS_COSTS = {
    {"unlabeled", 100.0f},
    {"road", 10.0f},
    {"sidewalk", 0.0f},
    {"building", 100.0f},
    {"wall", 100.0f},
    {"fence", 100.0f},
    {"pole", 50.0f},
    {"traffic light", 50.0f},
    {"traffic sign", 50.0f},
    {"vegetation", 30.0f},
    {"terrain", 20.0f},
    {"sky", 100.0f},
    {"person", 80.0f},
    {"rider", 80.0f},
    {"car", 100.0f},
    {"truck", 100.0f},
    {"bus", 100.0f},
    {"train", 100.0f},
    {"motorcycle", 80.0f},
    {"bicycle", 80.0f}
};

std::map<int, float> buildColorToCostMap()
{
    std::map<int, float> colorCostMap;
    for (size_t i = 0; i < COLOR_MAP.size(); ++i) {
        int red   = COLOR_MAP[i][0];
        int green = COLOR_MAP[i][1];
        int blue  = COLOR_MAP[i][2];

        // We assume the class name at the same index i is CLASSES[i].
        std::string className = CLASSES[i];
        float cost = 100.0f;
        auto it = CLASS_COSTS.find(className);
        if (it != CLASS_COSTS.end()) {
            cost = it->second;
        }

        int colorKey = (red << 16) + (green << 8) + blue; // (R,G,B)
        colorCostMap[colorKey] = cost;
    }
    return colorCostMap;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terrain_map_publisher");
    ros::NodeHandle nh;

    // 1. Load the color-segmented BEV image as BGR
    cv::Mat bevImage = cv::imread("/home/slsecret/PreferentialTerrainNavigation/src/bev/scripts/output/bev_image.png", cv::IMREAD_COLOR);
    if (bevImage.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }

    // 2. Build color->cost lookup
    std::map<int, float> colorCostMap = buildColorToCostMap();

    // 3. Create a single-channel float Mat for cost
    cv::Mat costImage(bevImage.rows, bevImage.cols, CV_32FC1);

    // 4. Convert from color to cost
    for (int row = 0; row < bevImage.rows; ++row) {
        const cv::Vec3b* rowPtr = bevImage.ptr<cv::Vec3b>(row);
        float* costPtr = costImage.ptr<float>(row);
        for (int col = 0; col < bevImage.cols; ++col) {
            cv::Vec3b cBGR = rowPtr[col];
            int b = cBGR[0];
            int g = cBGR[1];
            int r = cBGR[2];

            // (R,G,B) => same colorKey as in buildColorToCostMap()
            int colorKey = (r << 16) + (g << 8) + b;
            auto it = colorCostMap.find(colorKey);
            if (it != colorCostMap.end()) {
                costPtr[col] = it->second;
            } else {
                costPtr[col] = 100.0f; // default
            }
        }
    }

    // --- Convert costImage (cv::Mat) -> sensor_msgs::Image -> grid map layer. ---

    // 5. Prepare map geometry
    double pixelsPerMeter = 13.6;
    double resolution = 1.0 / pixelsPerMeter; 
    double mapWidth  = bevImage.cols * resolution;
    double mapHeight = bevImage.rows * resolution;

    std::cout << "bev image shape: " << bevImage.rows << "x" << bevImage.cols << std::endl;

    // 6. Create the GridMap
    grid_map::GridMap terrainMap({"terrain"});
    terrainMap.setFrameId("map");
    terrainMap.setGeometry(grid_map::Length(mapWidth, mapHeight),
                           resolution,
                           grid_map::Position(0.0, 0.0));

    // 7. Convert cv::Mat -> sensor_msgs::Image (using cv_bridge).
    //    We'll assume "32FC1" encoding for your costImage.
    cv::Mat costImage8u;
    costImage.convertTo(
        costImage8u,  // output
        CV_8UC1,      // target type
        255.0/100.0,  // alpha scale factor
        0.0           // beta offset
    );
    cv_bridge::CvImage cvImage(std_msgs::Header(), "mono8", costImage8u);
    sensor_msgs::Image costImageMsg;
    std::cout << "cv image shape: " << cvImage.image.rows << "x" << cvImage.image.cols << std::endl;
    cvImage.toImageMsg(costImageMsg);

    // 8. Now pass sensor_msgs::Image to addLayerFromImage()
    grid_map::GridMapRosConverter::addLayerFromImage(
        costImageMsg,    // sensor_msgs::Image
        "terrain",
        terrainMap,
        0.0,    // lowerValue in grid_map
        100.0,  // upperValue in grid_map
        0.5     // alphaThreshold (not really used in single-channel, but required in signature)
    );

    // 9. Publish the resulting grid map
    ros::Publisher gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Rate rate(1.0);

    while (ros::ok()) {
        terrainMap.setTimestamp(ros::Time::now().toNSec());

        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(terrainMap, msg);
        gridMapPub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
