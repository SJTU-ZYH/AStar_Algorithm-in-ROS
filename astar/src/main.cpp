#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "Astar.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
MapParamNode MapParam;
Mat Maptest;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;

// Parameter
double InflateRadius;
int map_flag = 0;
int find_flag = 0;
int start_flag = 0;
int target_flag = 0;

void World2MapGrid(MapParamNode& MapParam, Point2d& src_point, Point& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    Mat P_dst = MapParam.Rotation.inv() * (P_src - MapParam.Translation);

    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = MapParam.height - 1 - round(P_dst.at<double>(1, 0));
}
void MapGrid2world(MapParamNode& MapParam, Point& src_point, Point2d& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, MapParam.height - 1 - src_point.y), CV_64FC1);

    Mat P_dst = MapParam.Rotation * P_src + MapParam.Translation;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

void MapCallback(const nav_msgs::OccupancyGrid& msg)
{

    // Get the parameters of map
    MapParam.resolution = msg.info.resolution;
    MapParam.height = msg.info.height;
    MapParam.width = msg.info.width;
    // The origin of the MapGrid is on the bottom left corner of the map
    MapParam.x = msg.info.origin.position.x;
    MapParam.y = msg.info.origin.position.y;


    // Calculate the pose of map with respect to the world of rviz
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double theta = yaw;

    //从rviz上所给定的起点和终点坐标是真实世界坐标系下的位置，需要转化为地图坐标下的表示
    //MapParam.Rotation MapParam.Translation 用于该变换
    MapParam.Rotation = Mat::zeros(2,2, CV_64FC1);
    MapParam.Rotation.at<double>(0, 0) = MapParam.resolution * cos(theta);
    MapParam.Rotation.at<double>(0, 1) = MapParam.resolution * sin(-theta);
    MapParam.Rotation.at<double>(1, 0) = MapParam.resolution * sin(theta);
    MapParam.Rotation.at<double>(1, 1) = MapParam.resolution * cos(theta);
    MapParam.Translation = Mat(Vec2d(MapParam.x, MapParam.y), CV_64FC1);

    cout<<"Map:"<<endl;
    cout<<"MapParam.height:"<<MapParam.height<<endl;
    cout<<"MapParam.width:"<<MapParam.width<<endl;

    Mat Map(MapParam.height, MapParam.width, CV_8UC1);
    Maptest = Map;
    int GridFlag;
    for(int i = 0; i < MapParam.height; i++)
    {
        for(int j = 0; j < MapParam.width; j++)
        {
            GridFlag = msg.data[i * MapParam.width + j];
            GridFlag = (GridFlag < 0) ? 100 : GridFlag; // set Unknown to 0
            Map.at<uchar>(MapParam.height-i-1,j) = 255 - round(GridFlag * 255.0 / 100.0);
        }
    }
    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / MapParam.resolution);
    // ROS_INFO("Map_Judge");
    if(Map.empty()) ROS_INFO("Map Empty!!!");
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    int OccProb;
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    // cout << MapParam.height << MapParam.width << endl;
    for(int i=0;i<MapParam.height;i++)
    {
        for(int j=0;j<MapParam.width;j++)
        {
            OccProb = Mask.at<uchar>(MapParam.height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }
    
    // Change flag
    map_flag = 1;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    World2MapGrid(MapParam,src_point, MapParam.StartPoint);
    cout<<"StartPoint:"<<MapParam.StartPoint<<endl;

    start_flag = 1;
    find_flag = 1;
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    World2MapGrid(MapParam,src_point, MapParam.TargetPoint);
    int p =Maptest.at<uchar>(MapParam.TargetPoint.x, MapParam.TargetPoint.y);
    // cout<<"flag:"<<p<<endl;
    MapGrid2world(MapParam,MapParam.TargetPoint,src_point);
    // cout<<"TargetPoint world:"<<src_point<<endl;
    cout<<"TargetPoint:"<<MapParam.TargetPoint<<endl;

    target_flag = 1;
    find_flag = 1;
}
void PathGrid2world(MapParamNode& MapParam, vector<Point>& PathList, nav_msgs::Path& plan_path)
{
    plan_path.header.stamp = ros::Time::now();
    plan_path.header.frame_id = "map";
    plan_path.poses.clear();
    for(int i=0;i<PathList.size();i++)
    {
        Point2d dst_point;
        // ROS_INFO("1");
        MapGrid2world(MapParam,PathList[i], dst_point);
        // ROS_INFO("2");
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation.w = 1.0;
        // ROS_INFO("3");
        plan_path.poses.push_back(pose_stamped);
        // ROS_INFO("4");
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    // geometry_msgs::PointStamped astar_step;

    // Parameter
    n_priv.param<bool>("Euclidean", config.Euclidean, true);
    n_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    n_priv.param<double>("InflateRadius", InflateRadius, -1);

    // Subscribe topics
    ros::Subscriber Map_sub = n.subscribe("map", 10, MapCallback);
    ros::Subscriber StarPoint_sub = n.subscribe("initialpose", 10, StartPointCallback);
    ros::Subscriber TargetPoint_sub = n.subscribe("move_base_simple/goal", 10, TargetPointtCallback);

    // Publisher topics
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/NavfnROS/nav_path", 10);
    ros::Publisher mask_pub = n.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    ros::Rate loop_rate(10);
    nav_msgs::Path plan_path;

    vector<Point> PathList;
    while(ros::ok())
    {
        if (map_flag && start_flag && target_flag)
        {
            astar.PathPlanning(MapParam.StartPoint, MapParam.TargetPoint, PathList);
            if(!PathList.empty())
            {
                // ROS_INFO("FIND_PATH");
                if(find_flag){
                    ROS_INFO("FIND_PATH");
                    cout << "The PATH length is:" << PathList.size() << endl;
                    find_flag = 0;
                }
                PathGrid2world(MapParam, PathList, plan_path);
                path_pub.publish(plan_path);
            }
            else{
                ROS_INFO("NO_PATH");
            }
        }

        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
