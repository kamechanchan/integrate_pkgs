#include <w2d_to_3d_ros/exec_yolo.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init_exec_yolo");
    ros::NodeHandle nh;
    Exec_yolo yolo(nh);
    ros::spin();
}