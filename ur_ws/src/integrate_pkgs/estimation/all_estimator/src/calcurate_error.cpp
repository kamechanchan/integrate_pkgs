#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

tf2_ros::Buffer *buf;
tf2_ros::TransformListener *listener;

void tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &trans)
{
    while (1) {
        try 
        {
            trans = buf->lookupTransform(source_frame, target_frame, ros::Time(0));
            ROS_INFO_ONCE("I got transfer");
            break;
        }
        catch (tf2::TransformException &e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "error_calcurate");
    buf = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buf);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string common_source_frame, groud_truth_name, estimate_name;
    pnh.getParam("common_source_frame", common_source_frame);
    pnh.getParam("ground_truth_name", groud_truth_name);
    pnh.getParam("estimate_name", estimate_name);
    geometry_msgs::TransformStamped ground_truth_tf, estimate_tf;
    tf_get(common_source_frame, groud_truth_name, ground_truth_tf);
    tf_get(common_source_frame, estimate_name, estimate_tf);
    double x, y, z, roll, pitch, yaw;
    x = ground_truth_tf.transform.translation.x - estimate_tf.transform.translation.x;
    y = ground_truth_tf.transform.translation.y - estimate_tf.transform.translation.y;
    z = ground_truth_tf.transform.translation.z - estimate_tf.transform.translation.z;
    tf2::Quaternion g_q, e_q, error_qu;
    tf2::convert(ground_truth_tf.transform.rotation, g_q);
    tf2::convert(estimate_tf.transform.rotation, e_q);
    error_qu = g_q * e_q.inverse();
    tf2::getEulerYPR(error_qu, yaw, pitch, roll);
    yaw = yaw * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    roll = roll * 180 / M_PI;
    ROS_INFO_STREAM("x: " <<  x << "  y: " << y << "    z: " << z << "    angle: " << error_qu.getAngle() * 180 / M_PI);
    ROS_INFO_STREAM("roll: " << roll << "   pitch: " << pitch << "    yaw: " << yaw);
    return 0;

}