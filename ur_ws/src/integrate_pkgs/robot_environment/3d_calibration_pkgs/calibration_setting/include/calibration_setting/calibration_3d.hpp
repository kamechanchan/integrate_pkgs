#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/convert.h>

class Calibration_3d
{
public:
    Calibration_3d(ros::NodeHandle);
    geometry_msgs::TransformStamped get_tf(std::string, std::string);
    void parameter_set();
    tf2::Quaternion convert_quat(tf2::Quaternion, tf2::Quaternion, double);
    void try_1();
    void try_2();
    void send_tsuchida_tf();
    void send_sensor_tf();
private:
    std::string ar_frame_, true_frame_, base_frame_, sensor_frame_;
    geometry_msgs::TransformStamped ar_pose_, true_pose_, sensor_pose_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    tf2_ros::TransformBroadcaster dynamic_br_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    double q_x_para_, q_y_para_, q_z_para_, x_para_, y_para_, z_para_;
};