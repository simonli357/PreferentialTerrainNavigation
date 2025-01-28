#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include "AStarPlanner.hpp"
#include "mapping/TerrainMap.hpp"
#include "mapping/Constants.h"
#include "mapping/ImageToCost.hpp"

class Controller {
public:
    Controller(ros::NodeHandle &nh): nh_(nh)
    {
        // Initialize publishers
        map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("terrain_map", 1);
        path_pub_ = nh_.advertise<nav_msgs::Path>("planned_path", 1);

        // Initialize subscriber to the BEV image topic
        bev_image_sub_ = nh_.subscribe("bev_image", 1, &Controller::bevImageCallback, this);

        ROS_INFO("Controller initialized and ready to process BEV images.");
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber bev_image_sub_;

    ImageToCost image_to_cost_converter_;
    TerrainMap global_map_;
    AStarPlanner planner_;

    double robot_x_ = 50.0; // Default robot position (can be parameterized)
    double robot_y_ = 30.0;
    double robot_yaw_ = M_PI / 4; // Default robot orientation

    cv::Mat bev_image_;

    void bevImageCallback(const sensor_msgs::ImageConstPtr &msg) {
        // Convert the ROS image message to an OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        bev_image_ = cv_ptr->image;
        if (bev_image_.empty()) {
            ROS_ERROR("Received an empty image!");
            return;
        }

        // Convert image to cost map
        auto& local_map = image_to_cost_converter_.convert(bev_image_);

        // Update the global map
        global_map_.updateGlobalMap(local_map, robot_x_, robot_y_, robot_yaw_);
        auto& map_ = global_map_.getMap();

        // check the max and min values of the cost map
        double min, max;
        for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
            const auto& index = *it;
            const auto& cost = map_.at("terrainCost", index);
            if (cost < min) min = cost;
            if (cost > max) max = cost;
        }
        ROS_INFO("Cost map min: %f, max: %f", min, max);

        // Plan the path
        // grid_map::Index start_index(0, 0);
        // grid_map::Index goal_index(460, 415);
        // auto path_indices = planner_.plan(map_, "terrainCost", start_index, goal_index);
        auto start_pos = grid_map::Position(54.0, 14.0);
        auto goal_pos = grid_map::Position(90.0, 46.0);
        auto path_indices = planner_.planFromPosition(map_, "terrainCost", start_pos, goal_pos);

        // for (int i = 0; i < path_indices.size(); i++) {
        //     const auto& index = path_indices[i];
        //     const auto& cost = map_.at("terrainCost", index);
        //     grid_map::Position pos;
        //     map_.getPosition(index, pos);
        //     ROS_INFO("Path cell %d: cost %f, x %f, y %f", i, cost, pos.x(), pos.y());
        // }

        // Convert results to ROS messages
        auto path_msg = planner_.toPathMsg(path_indices, map_, "map");
        auto grid_map_msg = planner_.toGridMapMsg(map_);

        map_pub_.publish(grid_map_msg);
        path_pub_.publish(path_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "grid_map_a_star_publisher");
    ros::NodeHandle nh;
    Controller controller(nh);
    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
