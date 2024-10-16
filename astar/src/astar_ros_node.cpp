/**
 * @file astar_ros_node.cpp
 * @author Pablo Gutiérrez (pablo.gutierrez@htwg-konstanz.de)
 * @brief 
 * @version 0.1
 * @date 2024-10-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Astar/astar_ros_node.hpp"

using namespace pathplanning;

AstarROSNode::AstarROSNode():
Node("astar_node")
{

    this->declare_parameter<bool>("Euclidean", true);
    this->declare_parameter<bool>("ros_service", true);
    this->declare_parameter<int>("OccupyThresh", -1);
    this->declare_parameter<double>("InflateRadius", -1.0);
    this->declare_parameter<int>("rate", -1);

    config.Euclidean = this->get_parameter("Euclidean").as_bool();
    config.OccupyThresh = this->get_parameter("OccupyThresh").as_int();
    InflateRadius = this->get_parameter("InflateRadius").as_double();
    rate = this->get_parameter("rate").as_int();
    ros_service = this->get_parameter("ros_service").as_bool();

    //Services
    if(ros_service)
    {
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        get_path_srv = this->create_service<path_msgs::srv::GetPath>(
                                            "/astar/get_path", 
                                            std::bind(&AstarROSNode::getPathCallback, this, _1, _2),
                                            rmw_qos_profile_services_default,
                                            service_cb_group_);

        startPoint_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        targetPoint_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    }
    else
    {
        // Receive the start point
        startPoint_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10,
            std::bind(&AstarROSNode::StartPointCallback, this, _1));

        // Receive the target point
        targetPoint_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(&AstarROSNode::TargetPointCallback, this, _1));

        //Timers
        timer_ = this->create_wall_timer(std::chrono::seconds(1/rate),std::bind(&AstarROSNode::timer_callback, this));

    }

    //Subscribe map
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 
        10, 
        std::bind(&AstarROSNode::MapCallback, this, _1));


    //Publishers
    mask_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/astar/mask", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/astar/nav_path", 10);

    OccGridParam = std::make_unique<OccupancyGridParam>();
    astar = std::make_unique<pathplanning::Astar>();

    RCLCPP_INFO(this->get_logger(), "Astar node successfully started!");

}

void AstarROSNode::getPathCallback(const std::shared_ptr<path_msgs::srv::GetPath::Request> request,
                                   std::shared_ptr<path_msgs::srv::GetPath::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Getting path...");

    //response->success = true;
    Point2d start_point = Point2d(request->init_pose.pose.position.x, request->init_pose.pose.position.y);
    Point2d goal_point = Point2d(request->goal_pose.pose.position.x, request->goal_pose.pose.position.y);
    OccGridParam->Map2ImageTransform(start_point, startPoint);
    OccGridParam->Map2ImageTransform(goal_point, targetPoint);

    geometry_msgs::msg::PoseWithCovarianceStamped startPointMsg;
    startPointMsg.header.stamp = Node::now();
    startPointMsg.header.frame_id = "map";
    startPointMsg.pose.pose.position.x = request->init_pose.pose.position.x;
    startPointMsg.pose.pose.position.y = request->init_pose.pose.position.y;
    startPointMsg.pose.pose.position.z = 0;
    startPointMsg.pose.pose.orientation = request->init_pose.pose.orientation;

    startPoint_pub->publish(startPointMsg);

    geometry_msgs::msg::PoseStamped goalPointMsg;
    goalPointMsg.header.stamp = Node::now();
    goalPointMsg.header.frame_id = "map";
    goalPointMsg.pose.position.x = request->goal_pose.pose.position.x;
    goalPointMsg.pose.position.y = request->goal_pose.pose.position.y;
    goalPointMsg.pose.position.z = 0;
    goalPointMsg.pose.orientation = request->goal_pose.pose.orientation;

    targetPoint_pub->publish(goalPointMsg);

    
    startpoint_flag = true;
    targetpoint_flag = true;

    if(map_flag)
    {
        start_flag = true;
        this->compute_path();
    }

    response->path = this->path;

}


void AstarROSNode::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Get parameter
    OccGridParam->GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam->height;
    int width = OccGridParam->width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg->data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam->resolution);
    astar->InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = Node::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg->info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;   
}

void AstarROSNode::StartPointCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    Point2d src_point = Point2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    OccGridParam->Map2ImageTransform(src_point, startPoint);

    // Set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

}


void AstarROSNode::TargetPointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Point2d src_point = Point2d(msg->pose.position.x, msg->pose.position.y);
    OccGridParam->Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
}

void AstarROSNode::compute_path()
{
    if(start_flag)
    {
        std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
        // start planning path
        std::vector<Point> PathList;
        astar->PathPlanning(startPoint, targetPoint, PathList);
        if(!PathList.empty())
        {
            path.header.stamp = Node::now();
            path.header.frame_id = "map";
            path.poses.clear();
            for (int i = 0; i < (int)PathList.size(); i++)
            {
                Point2d dst_point;
                OccGridParam->Image2MapTransform(PathList[i], dst_point);

                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.stamp = Node::now();
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = dst_point.x;
                pose_stamped.pose.position.y = dst_point.y;
                pose_stamped.pose.position.z = 0;
                path.poses.push_back(pose_stamped);
            }
            path_pub->publish(path);
            std::chrono::high_resolution_clock::time_point last_time = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::milliseconds>(last_time - current_time).count();
            RCLCPP_INFO(this->get_logger(), "Find a valid path successfully! Use %f [ms]", dt);

        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can not find a valid path");
        }

        // Set flag
        start_flag = false;

        if(map_flag)
        {
            // Publish occupancy grid mask
            mask_pub->publish(OccGridMask);
        }


    }

}


void AstarROSNode::timer_callback()
{
    this->compute_path();
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pathplanning::AstarROSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}