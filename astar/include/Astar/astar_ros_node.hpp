/**
 * @file astar_ros_node.hpp
 * @author Pablo Gutiérrez (pablo.gutierrez@htwg-konstanz.de)
 * @brief 
 * @version 0.1
 * @date 2024-10-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ASTAR_ROS_NODE_HPP_
#define ASTAR_ROS_NODE_HPP_

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "path_msgs/srv/get_path.hpp"
#include "path_msgs/srv/pose.hpp"
//#include <nav2_msgs/srv/is_path_valid.hpp>

#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"


using namespace cv;
using namespace std;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace pathplanning
{

class AstarROSNode: public rclcpp::Node
{
private:

  
  nav_msgs::msg::OccupancyGrid OccGridMask;
  nav_msgs::msg::Path path;
  pathplanning::AstarConfig config;
  std::unique_ptr<pathplanning::Astar> astar;
  std::unique_ptr<OccupancyGridParam> OccGridParam;
  Point startPoint, targetPoint;

  // Parameter
  double InflateRadius;
  bool map_flag;
  bool startpoint_flag;
  bool targetpoint_flag;
  bool start_flag;
  bool ros_service;
  int rate;

  std::chrono::high_resolution_clock::time_point wheel_last_time;
  std::chrono::steady_clock::time_point last_time;
  rclcpp::TimerBase::SharedPtr timer_;

  //Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr startPoint_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetPoint_sub;

  //Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr startPoint_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr targetPoint_pub;

  //Services
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<path_msgs::srv::GetPath>::SharedPtr get_path_srv;

  // Service callbacks
  void getPathCallback(const std::shared_ptr<path_msgs::srv::GetPath::Request> request,
                       std::shared_ptr<path_msgs::srv::GetPath::Response> response);

  // Subscriber callbacks
  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void StartPointCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void TargetPointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void compute_path();
  void timer_callback();


public:
  
  AstarROSNode();
  ~AstarROSNode(){}



};

} //end pathplanning namespace

#endif  // ASTAR_ROS_NODE_HPP_