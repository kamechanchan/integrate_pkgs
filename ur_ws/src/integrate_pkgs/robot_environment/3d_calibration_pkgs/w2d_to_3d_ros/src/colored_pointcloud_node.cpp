#include <w2d_to_3d_ros/colored_pointcloud.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init_node");
    ros::NodeHandle nh;
    Colored_PointCloud hajime(nh);
    ros::spin();
}