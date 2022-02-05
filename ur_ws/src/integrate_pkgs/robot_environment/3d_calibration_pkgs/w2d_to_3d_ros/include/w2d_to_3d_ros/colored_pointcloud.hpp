#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>

class Colored_PointCloud
{
public:
    Colored_PointCloud(ros::NodeHandle);
    void tf_get(std::string, std::string);
    // void InputCallback(sensor_msgs::CameraInfoConstPtr, sensor_msgs::ImageConstPtr, sensor_msgs::PointCloud2ConstPtr);
    void InputCallback(sensor_msgs::CameraInfoConstPtr, sensor_msgs::ImageConstPtr);

    void parameter_set();
    void colored_get(sensor_msgs::CameraInfo, sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZRGB>&, cv::Mat&);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> Sync_Sub_Type;
    template <typename T>
    void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
    {
        boost::shared_ptr<const T> message;
        message = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
        if (message != NULL) {
            final_message = *message;
        }
        message.reset();
    }
    cv::Point2d project3d_to_pixel(cv::Point3d, sensor_msgs::CameraInfo);
private:
    // message_filters::Subscriber<sensor_msgs::PointCloud2> *sensor_cloud_;
    message_filters::Synchronizer<Sync_Sub_Type> *sensor_sync_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle *pnh_;
    std::string source_frame_, target_frame_;
    pcl::PointCloud<pcl::PointXYZRGB> colored_pointcloud_;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud_;
    sensor_msgs::PointCloud2 output_cloud_;
    ros::Publisher output_pub;
    std::string inputcloud_topic_name_, outputcloud_topic_name_, camera_topic_name_, image_topic_name_;
    tf::StampedTransform transform_;
    cv::Mat draw_image_;
    std::ofstream *ofs;
    std::string paramter_output_file_path;
    double f_scale_, cx_scale_, cy_scale_;
    double fx, fy, tx, ty, cx, cy;
};