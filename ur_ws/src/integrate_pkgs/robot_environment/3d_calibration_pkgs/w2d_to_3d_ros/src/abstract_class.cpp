#include <w2d_to_3d_ros/abstract_class.hpp>


void cloud_abstract::set_input_cloud(const sensor_msgs::PointCloud2 input_cloud){
    input_ = input_cloud;
}

// handle:抽象クラス topic_name:点群をサブスクライブするトピック
cloud_operate_handle::cloud_operate_handle(ros::NodeHandle &nh, cloud_abstract* handle, std::string topic_name)
: handle_(handle),
sub_(nh.subscribe(topic_name, 10, &cloud_operate_handle::callback, this))
{
    // ROS_INFO_STREAM("11");
    nh_ = nh;
    // handle_ = handle;
    ROS_INFO_STREAM(topic_name);
    // ROS_INFO("%s", topic_name);
    // sub_ = nh.subscribe(topic_name, 10, &cloud_operate_handle::callback, this);
}

void cloud_operate_handle::callback(const sensor_msgs::PointCloud2 &msg)
{
    ROS_INFO_STREAM("msgissssssssss");
    handle_ -> set_input_cloud(msg);
    ROS_INFO_STREAM("ishiyama");
    handle_ -> operate();
    ROS_INFO_STREAM("kai");

    handle_ -> publish();
}