#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer *buffer;
tf2_ros::TransformListener *listener;

geometry_msgs::TransformStamped get_tf(std::string target, std::string source)
{
    geometry_msgs::TransformStamped final_tf;
    while (1)
    {
        try 
        {
            final_tf = buffer->lookupTransform(source, target, ros::Time(0));
            ROS_INFO_ONCE("get tf");
            break;
        }
        catch (tf2::TransformException &e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    return final_tf;
}

tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iyana_tf");
    buffer = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buffer);
    std::string ori_target, ori_source;
    geometry_msgs::TransformStamped original_tf, out_tf_1;
    ros::NodeHandle pnh("~");
    pnh.getParam("ori_target", ori_target);
    pnh.getParam("ori_source", ori_source);
    original_tf = get_tf(ori_target, ori_source);
    tf2::Quaternion ori_quat;
    tf2::convert(original_tf.transform.rotation, ori_quat);
    out_tf_1.header.frame_id = ori_source;
    tf2::Quaternion q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_z(0, 0, 1, 0);
    tf2::convert(convert_quat(q_z, ori_quat, (M_PI/2)+M_PI) * ori_quat, out_tf_1.transform.rotation);
    out_tf_1.transform.translation =original_tf.transform.translation;
    out_tf_1.header.stamp = ros::Time::now();
    out_tf_1.child_frame_id = "camera_depth_optical_frame";
    tf2_ros::StaticTransformBroadcaster br_;
    ros::Rate loop(10);
    while (ros::ok())
    {
        out_tf_1.header.stamp = ros::Time::now();
        br_.sendTransform(out_tf_1);
        loop.sleep();
    }
    return 0;
}