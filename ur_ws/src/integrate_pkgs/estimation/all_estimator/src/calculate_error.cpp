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


    // tf2::Quaternion q_zero(0, 0, 1, 0);
    // geometry_msgs::Transform trans = estimate_tf.transform;
    // tf2::Quaternion q_moto(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w);
    // tf2::Quaternion q_convert;
    // q_convert.setRPY(0, M_PI, 0);
    // tf2::Quaternion trans_ato, rotate_at;
    // trans_ato = q_moto * q_zero * q_moto.inverse();
    // rotate_at = q_convert * q_moto * q_convert.inverse();

    // tf2::Quaternion z_moto(0, 1, 0, 0), z_ato, quat;
    // tf2::Quaternion q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_z(0, 0, 1, 0);
    // z_ato = q_moto * z_moto * q_moto.inverse();
    // std::cout << z_ato[0] << "  " << z_ato[1] << "  " << z_ato[2] << std::endl;
    // tf2::Vector3 z_axis(z_ato[0], z_ato[1], z_ato[2]);
    // quat.setRotation(z_axis, M_PI);
    // q_moto = quat * q_moto;
    // rotate_at = q_moto;
    // tf2::Quaternion z_saki(0, 0, 1, 0);
    // z_ato = q_moto * z_saki * q_moto.inverse();
    // tf2::Vector3 z_axis1(z_ato[0], z_ato[1], z_ato[2]);
    // quat.setRotation(z_axis1, 3*M_PI/ 2);
    // q_moto = quat * q_moto;
    // // Arm_Move::convert_quat(q_y, q_moto, M_PI / 2);
    // q_moto = convert_quat(q_y, q_moto, -M_PI / 2)*q_moto;

    // rotate_at = q_moto;

    // estimate_tf.transform.translation.x = trans_ato[0]  + trans.translation.x;
    // estimate_tf.transform.translation.y = trans_ato[1] + trans.translation.y;
    // estimate_tf.transform.translation.z = trans_ato[2] + trans.translation.z;
    
    // estimate_tf.transform.rotation.x = rotate_at[0];
    // estimate_tf.transform.rotation.y = rotate_at[1];
    // estimate_tf.transform.rotation.z = rotate_at[2];
    // estimate_tf.transform.rotation.w = rotate_at[3];

    
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