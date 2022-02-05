#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

void callback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO_STREAM("get_image");
    cv_bridge::CvImagePtr input_bridge;
    if (msg != NULL) {
        input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    else {
        ROS_ERROR_STREAM("Fail to get image!!");
        return;
    }
    cv::Mat img;
    img = input_bridge->image;
    cv::imshow("mask_show", img);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ffeiei");
    ros::Subscriber sub;
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string image_topic = "/photoneo_center/rgb_texture";
    pnh.getParam("image_topic", image_topic);
    sub = nh.subscribe(image_topic, 10, callback);
    ros::spin();
    return 0;
}

