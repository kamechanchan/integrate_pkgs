#include <ros/ros.h>
#include <w2d_to_3d_ros/hayasa.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init");
    ros::NodeHandle nh;
    ros::Publisher pub_2;
    pub_2 = nh.advertise<w2d_to_3d_ros::hayasa>("shinya_2", 10);
    ros::Rate loop(10);
    w2d_to_3d_ros::hayasa msg;
    msg.count = 0;
    while (ros::ok())
    {
        
        msg.count++;
        msg.header.stamp = ros::Time::now();
        pub_2.publish(msg);
        loop.sleep();
    }
    return 0;
}