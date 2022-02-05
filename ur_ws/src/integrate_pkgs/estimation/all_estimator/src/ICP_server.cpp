#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <denso_msgs/dummy_pcl.h>
#include <denso_msgs/out_segmentation.h>

ros::NodeHandle *pnh;
std::string dummy_topic_name;
std::string publish_topic_name;
std::string instance_topic_name;
std::string frame_id_;

ros::Publisher dummy_pub;
ros::Publisher instance_pub;

void callback(denso_msgs::out_segmentationConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    ROS_INFO_STREAM("get msg");
    ROS_INFO_STREAM("cloud size is " << msg->x.size());
    int color_0 = 0;
    for (int i = 0; i < msg->x.size(); i++) {
        pcl::PointXYZRGB color;
        color.x = msg->x[i];
        color.y = msg->y[i];
        color.z = msg->z[i];
        color.r = 0;
        color.g = 255;
        color.b = 0;
        color_0++;
        cloud.push_back(color);
    }
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    instance_pub.publish(ros_msg);
    ROS_INFO_STREAM("instance 0: " << color_0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inti");
    ros::NodeHandle nh;
    pnh = new ros::NodeHandle("~");
    pnh->getParam("dummy_topic_name", dummy_topic_name);
    pnh->getParam("instance_topic_name", instance_topic_name);
    pnh->getParam("frame_id", frame_id_);
    instance_pub = nh.advertise<sensor_msgs::PointCloud2>(instance_topic_name, 10);
    ros::Subscriber sub;
    sub = nh.subscribe(dummy_topic_name, 10, callback);
    ros::spin();
    return 0;
}