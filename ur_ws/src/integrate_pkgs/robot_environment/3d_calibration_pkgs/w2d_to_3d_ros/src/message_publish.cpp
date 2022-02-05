#include <ros/ros.h>
#include <w2d_to_3d_ros/shinya.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init2");
    ros::NodeHandle nh;
    ros::Publisher pub_2;
    pub_2 = nh.advertise<w2d_to_3d_ros::shinya>("shinya_1", 10);
    w2d_to_3d_ros::shinya shn;
    shn.data = "tsuchida";
    
    ros::Rate loop(10);
    while (ros::ok())
    {
        shn.header.stamp = ros::Time::now();
        pub_2.publish(shn);
        loop.sleep();
    }
    return 0;
}