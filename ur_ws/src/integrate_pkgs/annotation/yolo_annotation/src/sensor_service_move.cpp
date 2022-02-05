#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
    ros::init(argc, argv, "tsuchdia_");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    tf2_ros::StaticTransformBroadcaster st_br;
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = "phoxi_camera";
    double x, y, z;
    std::random_device rd;
    std::default_random_engine eng(rd());
    geometry_msgs::TransformStamped origin_pose;
    tf2::Quaternion source_quat, q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_z(0, 0, 1, 0);

    tf2::convert(origin_pose.transform.rotation, source_quat);
    geometry_msgs::TransformStamped tf_pose;
    int count = 0;
    while (count <= 10) {
        
        double r = 1;
        std::uniform_real_distribution<> distr(-M_PI/6, M_PI/6);
        double angle = distr(eng);
        
        srv.request.model_state.pose.position.x = r*sin(angle);
        srv.request.model_state.pose.position.y = 0;
        srv.request.model_state.pose.position.z = r * cos(angle);

        tf2::Quaternion source(0, 0, 0, 1);
        tf2::Quaternion qu_optical = convert_quat(q_y, source_quat, -angle) * source_quat;
        tf2::Quaternion qu = convert_quat(q_y, source, angle) * source;
       
        tf2::convert(qu, srv.request.model_state.pose.orientation);
        tf2::convert(qu_optical, tf_pose.transform.rotation);
        // std::cout << "srv.request.model_state_pose_ori" << "  x: " << srv.request.model_state.pose.orientation.x << "  y: " << srv.request.model_state.pose.orientation.y
        // << "  z: " << srv.request.model_state.pose.orientation.z << "  w: " << srv.request.model_state.pose.orientation.w << std::endl;
        tf_pose.transform.translation.x = srv.request.model_state.pose.position.x;
        tf_pose.transform.translation.y = srv.request.model_state.pose.position.y;
        tf_pose.transform.translation.z = srv.request.model_state.pose.position.z;
        tf2::convert(qu * convert_quat(q_y, qu, M_PI) * convert_quat(q_y, qu, -M_PI_2), tf_pose.transform.rotation);
        tf_pose.child_frame_id = "photoneo_center";
        tf_pose.header.frame_id = "base_link";
        tf_pose.header.stamp = ros::Time::now();
        
        ros::Rate loop(15);
        int naka_count = 0;
        
        tf_pose.header.stamp = ros::Time::now();
        client.call(srv);
        std::cout << srv.response.status_message << std::endl;
        std::cout << srv.response.success << std::endl;
        ros::Duration(0.7).sleep();
        st_br.sendTransform(tf_pose);
        
    
        
        
        count++;
    }
    tf_pose.header.frame_id = "tsuchida";
    tf_pose.child_frame_id = "photoneo_center_optical_frame";
    st_br.sendTransform(tf_pose);
    return 0;

    
}