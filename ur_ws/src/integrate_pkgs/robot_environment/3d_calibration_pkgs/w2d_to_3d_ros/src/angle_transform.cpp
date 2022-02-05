#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#define PI 3.141592

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angle_convert");
    tf2::Quaternion q_rot;
    double roll = PI, pitch = 0, yaw = PI / 2;
    q_rot.setRPY(roll, pitch, yaw);
    tf2::Quaternion quat(0.1, 0, -2, 0);
    
    tf2::Quaternion quat_xyz_rpy = q_rot * quat * q_rot.inverse();
    // tf::Quaternion q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
    // tf::Quaternion q_xyz(0.1, 0, 1, 0);
    // tf::Quaternion quat_xyz_rpy = q_rot * q_xyz * q_rot.inverse();
    ROS_INFO_STREAM("x: " << q_rot.x() << "  y: " << q_rot.y() << "  z: " << q_rot.z());
    ROS_INFO_STREAM("x: " << quat_xyz_rpy.x() << "  y: " << quat_xyz_rpy.y() << "  z: " << quat_xyz_rpy.z());
    return 0;

}