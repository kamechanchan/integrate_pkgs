#include <w2d_to_3d_ros/ano_colored_pointcloud.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "colored");
    // ROS_INFO_STREAM("feei");
    ros::NodeHandle nh;
    bool not_finish = true;
    nh.setParam("not_finish", true);
    Annotation_yolo yolo(nh);
    ros::Rate loop(10);
    while (not_finish) {
        ros::spinOnce();
        nh.getParam("not_finish", not_finish);
        loop.sleep();
    }
    
    
}