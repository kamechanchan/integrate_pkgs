#include <w2d_to_3d_ros/ano_and_exec.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tuschda_inti");
    ros::NodeHandle nh;
    Ano_and_Exec ano_and(nh);
    ros::spin();
}