#include <w2d_to_3d_ros/abstract_class.hpp>
#include <w2d_to_3d_ros/cloud_clustering.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "main_pcl");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string topic_name;

    pnh.getParam("topic_name", topic_name);
    ROS_INFO_STREAM("main: " << topic_name);

    while(ros::ok()){
        cloud_operate_handle handle_planar(nh, new Clustering(nh), topic_name);
        ros::spin();
    }

}