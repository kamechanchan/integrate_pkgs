#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <denso_srvs/tf_quaternion_service.h>

tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

bool service_callback(denso_srvs::tf_quaternion_service::Request &req, denso_srvs::tf_quaternion_service::Response &res)
{
    tf2::Quaternion q_moto, q_z(0, 0, 1, 0);
    tf2::convert(req.input_tf.rotation, q_moto);
    q_moto = convert_quat(q_z, q_moto, 3* M_PI / 2) * q_moto;
    res.output_tf.translation = req.input_tf.translation;
    tf2::convert(q_moto, res.output_tf.rotation);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quartenion_servic");
    ros::NodeHandle nh;
    ros::ServiceServer serve = nh.advertiseService("quaternion_service", service_callback);
    ros::spin();
    return 0;
}