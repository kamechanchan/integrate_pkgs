#include <all_estimator/yolo_server_2D.h>

std::string image_topic;
GetDataNode::GetDataNode()
{
    ROS_INFO_STREAM("start");
    photoneo_img_sub_ = nh_.subscribe(image_topic, 1000, &GetDataNode::callback_, this);
    server_ = nh_.advertiseService("ishiyama_input_data", &GetDataNode::inputData, this);
}

void GetDataNode::callback_(const sensor_msgs::Image& msg){
    data_ = msg;
    ROS_INFO_STREAM("get_photoneo_image");
}

bool GetDataNode::inputData(denso_srvs::input_data::Request &req, denso_srvs::input_data::Response &res){
    // res.out_img = req.in_img;
    res.out_img = data_;
    ROS_INFO_STREAM("success");
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ishinyam_1");
    ros::NodeHandle pnh("~");
    pnh.getParam("image_topic", image_topic);
    GetDataNode tanomuze;
    ros::spin();
}
