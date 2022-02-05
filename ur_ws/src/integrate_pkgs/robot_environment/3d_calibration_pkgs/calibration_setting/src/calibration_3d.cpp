#include <calibration_setting/calibration_3d.hpp>

Calibration_3d::Calibration_3d(ros::NodeHandle nh)
: buffer_(), listener_(buffer_), nh_(nh), pnh_("~")
{
    parameter_set();
    ros::Rate loop(10);
    send_tsuchida_tf();
    try_2();
    // try_2();
    while (ros::ok()) {
        
        send_sensor_tf();
        loop.sleep();
    }
    

}

void Calibration_3d::send_tsuchida_tf()
{
    ar_pose_ = get_tf(ar_frame_, sensor_frame_);
    geometry_msgs::Transform itiji;
    tf2::Quaternion q_qr_1, q_x(1, 0, 0, 0), q_z(0, 0, 1, 0), q_y(0, 1, 0, 0);
    tf2::convert(ar_pose_.transform.rotation, q_qr_1);
    

    // q_qr_1 = convert_quat(q_x, q_qr_1, -M_PI / 40) * q_qr_1;
    // q_qr_1 = convert_quat(q_z, q_qr_1, 0) * q_qr_1;
    // q_qr_1 = convert_quat(q_y, q_qr_1, +M_PI / 12) * q_qr_1;
    // itiji.translation.x = ar_pose_.transform.translation.x - 0.1;
    // itiji.translation.y = ar_pose_.transform.translation.y + 0.013;
    // itiji.translation.z = ar_pose_.transform.translation.z - 0.0;

    // q_qr_1 = convert_quat(q_x, q_qr_1, -M_PI / 20) * q_qr_1;
    // q_qr_1 = convert_quat(q_z, q_qr_1, 0) * q_qr_1;
    // q_qr_1 = convert_quat(q_y, q_qr_1, 0) * q_qr_1;
    // itiji.translation.x = ar_pose_.transform.translation.x - 0.105;
    // itiji.translation.y = ar_pose_.transform.translation.y + 0.013;
    // itiji.translation.z = ar_pose_.transform.translation.z - 0.015;


    q_qr_1 = convert_quat(q_x, q_qr_1, q_x_para_ * M_PI) * q_qr_1;
    q_qr_1 = convert_quat(q_z, q_qr_1, q_z_para_ * M_PI) * q_qr_1;
    q_qr_1 = convert_quat(q_y, q_qr_1, q_y_para_ * M_PI) * q_qr_1;
    itiji.translation.x = ar_pose_.transform.translation.x + x_para_;
    itiji.translation.y = ar_pose_.transform.translation.y + y_para_;
    itiji.translation.z = ar_pose_.transform.translation.z + z_para_;


    tf2::convert(q_qr_1, itiji.rotation);
    geometry_msgs::TransformStamped itiji_frame;
    itiji_frame.child_frame_id = "tsuchida";
    itiji_frame.header.frame_id = sensor_frame_;
    itiji_frame.header.stamp = ros::Time::now();
    itiji_frame.transform = itiji;
    static_br_.sendTransform(itiji_frame);
    ROS_INFO_STREAM("senfo tsuchida");
}
void Calibration_3d::try_1()
{
    ar_pose_ = get_tf(ar_frame_, base_frame_);
    true_pose_ = get_tf(true_frame_, base_frame_);
    sensor_pose_ = get_tf(sensor_frame_, base_frame_);
    geometry_msgs::Transform sabun, itiji;
    // sabun.translation.x = true_pose_.transform.translation.x - ar_pose_.transform.translation.x + sensor_pose_.transform.translation.x;
    // sabun.translation.y = true_pose_.transform.translation.y - ar_pose_.transform.translation.y + sensor_pose_.transform.translation.y;
    // sabun.translation.z = true_pose_.transform.translation.z - ar_pose_.transform.translation.z + sensor_pose_.transform.translation.z;
    sabun.translation.x = -ar_pose_.transform.translation.x + sensor_pose_.transform.translation.x;
    sabun.translation.y = - ar_pose_.transform.translation.y + sensor_pose_.transform.translation.y;
    sabun.translation.z = - ar_pose_.transform.translation.z + sensor_pose_.transform.translation.z;
    itiji.translation = ar_pose_.transform.translation;
    
    tf2::Quaternion q_ar, q_true, q_sabun, q_sensor, q_z(0, 0, 1, 0), q_qr_1, q_x(1, 0, 0, 0);
    tf2::convert(ar_pose_.transform.rotation, q_qr_1);
    q_qr_1 = convert_quat(q_x, q_qr_1, -M_PI / 35) * q_qr_1;
    tf2::convert(q_qr_1, itiji.rotation);
    tf2::convert(ar_pose_.transform.rotation, q_ar);
    q_ar = convert_quat(q_z, q_ar, M_1_PI);
    tf2::convert(true_pose_.transform.rotation, q_true);
    tf2::convert(sensor_pose_.transform.rotation, q_sensor);
    q_sabun = q_ar.inverse() * q_true;
    tf2::convert(q_sensor, sabun.rotation);
    geometry_msgs::TransformStamped sensor_final, itiji_frame;
    sensor_final.child_frame_id = sensor_frame_;
    sensor_final.header.frame_id = true_frame_;
    sensor_final.header.stamp = ros::Time::now();
    sensor_final.transform = sabun;
    itiji_frame.child_frame_id = "tsuchida";
    itiji_frame.header.frame_id = base_frame_;
    itiji_frame.header.stamp = ros::Time::now();
    itiji_frame.transform = itiji;
    static_br_.sendTransform(itiji_frame);
    static_br_.sendTransform(sensor_final);
    ROS_INFO_STREAM("send tf");
}

void Calibration_3d::send_sensor_tf()
{
    geometry_msgs::TransformStamped final_mae, sensor_final;
    final_mae = get_tf("tsufe", base_frame_);
    sensor_final.child_frame_id = sensor_frame_;
    sensor_final.header.frame_id = base_frame_;
    sensor_final.header.stamp = ros::Time::now();
    sensor_final.transform = final_mae.transform;
    static_br_.sendTransform(sensor_final);
}
void Calibration_3d::try_2()
{
    sensor_pose_ = get_tf(sensor_frame_, "tsuchida");
    geometry_msgs::TransformStamped sensor_final;
    sensor_final.child_frame_id = "tsufe";
    sensor_final.header.frame_id = true_frame_;
    sensor_final.header.stamp = ros::Time::now();
    sensor_final.transform = sensor_pose_.transform;
    static_br_.sendTransform(sensor_final);
}

geometry_msgs::TransformStamped Calibration_3d::get_tf(std::string target, std::string source)
{
    geometry_msgs::TransformStamped final_tf;
    while (1)
    {
        try
        {
            final_tf = buffer_.lookupTransform(source, target, ros::Time(0));
            ROS_INFO_ONCE("get_tf");
            break;
        }
        catch(const std::exception& e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;    
        }
    }
    return final_tf;
}

tf2::Quaternion Calibration_3d::convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

void Calibration_3d::parameter_set()
{
    pnh_.getParam("ar_frame", ar_frame_);
    pnh_.getParam("true_frame", true_frame_);
    pnh_.getParam("base_frame", base_frame_);
    pnh_.getParam("sensor_frame", sensor_frame_);
    pnh_.getParam("q_x", q_x_para_);
    pnh_.getParam("q_y", q_y_para_);
    pnh_.getParam("q_z", q_z_para_);
    pnh_.getParam("x_para", x_para_);
    pnh_.getParam("y_para", y_para_);
    pnh_.getParam("z_para", z_para_);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;
    Calibration_3d calib(nh);
    
}