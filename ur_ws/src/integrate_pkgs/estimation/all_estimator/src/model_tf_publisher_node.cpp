#include <all_estimator/model_tf_publisher.h>

using model_tf_publisher::ModelTf;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_tf_publsher");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ModelTf br(nh);
    ROS_INFO_STREAM("Publish model TF");
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        br.broadcastModelTF();
        loop_rate.sleep();
    }
    return 0;
}