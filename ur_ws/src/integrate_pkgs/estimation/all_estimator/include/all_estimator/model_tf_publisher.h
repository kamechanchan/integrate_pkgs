#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace model_tf_publisher
{
    class ModelTf
    {
    public:
        ModelTf(ros::NodeHandle&);
        void modelstatesCallback(const gazebo_msgs::ModelStates::ConstPtr&);
        int getModelId();
        void broadcastModelTF();
        void get_photoneo_tf(std::string);
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle *pnh;
        std::string src_frame_, urdf_model_name_;
        ros::Subscriber model_state_sub_;
        std::vector<std::string> model_names_;
        std::vector<geometry_msgs::Pose> model_poses_;
        geometry_msgs::TransformStamped tf_stamp_;
        tf::TransformBroadcaster br_;
        tf2_ros::TransformListener *TfListener;
        tf2_ros::Buffer TfBuffer;
        geometry_msgs::TransformStamped photoneo_trans;

    };
}