#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class cloud_abstract
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
public:
    virtual void operate() = 0;
    virtual void publish() = 0;
    void set_input_cloud(const sensor_msgs::PointCloud2);
protected:
    sensor_msgs::PointCloud2 input_;
    sensor_msgs::PointCloud2 output_rosmsg_;
};


class cloud_operate_handle
{
private:
    ros::NodeHandle nh_;
    cloud_abstract* handle_;
    ros::Subscriber sub_;

public:
    cloud_operate_handle(ros::NodeHandle&, cloud_abstract*, std::string);
    void callback(const sensor_msgs::PointCloud2&);
};
