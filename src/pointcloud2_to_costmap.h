// Header file for pointcloud2_to_costmap.cpp

// C++
#include <stdio.h>
#include <iostream>
#include <vector>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

// Config
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>


class PointCloudType{

    private:

    // union to convert data from from 4 bytes to Float32
    union UStuff {
        float   f;
        unsigned char   c[0];
    };
    // func to convert data from from 4 bytes to Float32 
    float Convert(unsigned int (&c) [4]);

    // publisher and msg for one group of PointCloud2
    ros::Publisher pub;
    sensor_msgs::PointCloud2 _msg;

    public:

    std::string group;
    std::vector<int> values;
    PointCloudType(ros::NodeHandle *nh, std::string group_){
        group = group_;
        pub = nh->advertise<sensor_msgs::PointCloud2>(group, 10);    
    }
    void send_msg(const sensor_msgs::PointCloud2ConstPtr& msg);
    
};


class PointCloudConverter{
    
    private:

    // subsriber for input data
    ros::Subscriber sub;

    // config.yaml
    struct Config{
        std::string name;
        int value;
        std::string group;
    };
    void read_config(ros::NodeHandle *nh);
    void print_config(bool print_conf);
    std::string param_name = "types";
    std::vector<Config> config;
    std::vector<PointCloudType> types;

    // create groups of PointCloud2
    void create_types(ros::NodeHandle *nh);
    void print_types();
    
    public:

    PointCloudConverter(ros::NodeHandle *nh){
        read_config(nh);
        sub = nh->subscribe("/stereo_depth/point_cloud", 1, &PointCloudConverter::Callback, this);
        create_types(nh);
        print_types();
    }
    void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};