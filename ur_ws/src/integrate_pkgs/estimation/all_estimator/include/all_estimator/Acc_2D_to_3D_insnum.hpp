#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <denso_msgs/object_kiriwake.h>
#include <denso_msgs/yolo_bridge.h>
#include <denso_srvs/bounding.h>
#include <denso_srvs/input_bridge.h>
#include <denso_srvs/Acc_bridge.h>

class YoloBridge
{
public:
    YoloBridge(ros::NodeHandle&);
    void major();
    void parameter_set();
    
    
    void box_get(sensor_msgs::CameraInfo, sensor_msgs::Image, std::vector<cv::Point3d>, cv::Mat&, std::vector<std::vector<cv::Point2d>>&);
   
   
    cv::Point2d project3d_to_pixel(cv::Point3d, sensor_msgs::CameraInfo);
    cv::Point2d project3d_to_pixel_origin(cv::Point3d, sensor_msgs::CameraInfo);
    void paramter_set_bara();
    void rotation_convert(geometry_msgs::TransformStamped, std::vector<geometry_msgs::TransformStamped>, std::vector<cv::Point3d>&);
    void get_original_image(sensor_msgs::Image, cv::Mat&);
    template <typename T>
    void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
    {
        boost::shared_ptr<const T> share;
        share = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
        if (share != NULL) {
            final_message = *share;
        }
        share.reset();
    }
    std::vector<std::vector<int>> write_instance(std::vector<std::vector<cv::Point2d>>, cv::Mat draw_IMG, std::vector<int>&);
    template <class T>
    void swap(T &yes, T &we)
    {
        T t = yes;
        yes = we;
        we = t;
    }

    // void hurui(pcl::PointCloud<pcl::PointXYZ>, std::vector<std::vector<int>>, sensor_msgs::Image, sensor_msgs::CameraInfo, pcl::PointCloud<pcl::PointXYZRGB>&);
    void hurui(pcl::PointCloud<pcl::PointXYZ>, std::vector<std::vector<int>>, sensor_msgs::Image, sensor_msgs::CameraInfo, pcl::PointCloud<pcl::PointXYZ>&);
    void callback(sensor_msgs::CameraInfoConstPtr cam_msgs);
    bool inputData(denso_srvs::bounding::Request &req, denso_srvs::bounding::Response &res);
    bool bridge(denso_srvs::Acc_bridge::Request &req, denso_srvs::Acc_bridge::Response &res);
    void srv_init(std::vector<int>);

private:
    ros::Subscriber sub_;
    ros::ServiceClient client_;
    ros::ServiceServer server_;
    std::vector<std::vector<int>> image_instance_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher output_pub_;
    std::string inputcloud_topic_name_;
    std::string output_topic_name_;
    std::string YOLO_topic_name_;
    std::string camera_topic_name_;
    std::string frame_id_;
    int instance_number_;
    tf2_ros::TransformListener lister_;
    tf2_ros::Buffer buffer_;
    sensor_msgs::PointCloud2 output_cloud_msgs_;
    double f_scale_, cx_scale_, cy_scale_;
    double fx_, fy_, tx_, ty_, cx_, cy_;
    float radious_;
    bool write_is_ok_;
    std::string image_dir_name_, filebasename_, model_name_, label_dir_name_, boxes_dir_name_;
    int the_number_of_data;
    int timespan_;
    sensor_msgs::CameraInfo cinfo_;
    pcl::PointCloud<pcl::PointXYZ> color_cloud_;
    pcl::PointCloud<pcl::PointXYZ> trans_cloud_;
    sensor_msgs::PointCloud2 cloud_msgs_;
    denso_srvs::Acc_bridge srv_;
    int Bbox_num_;
};