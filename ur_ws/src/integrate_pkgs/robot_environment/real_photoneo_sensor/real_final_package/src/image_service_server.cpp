#include <ros/ros.h>
#include <denso_srvs/sensor_data_service.h>
#include <sensor_msgs/Image.h>



class Receiver_Message
{
public:
    Receiver_Message(ros::NodeHandle &nh) : pnh_("~"), nh_(nh)
    {
        parameter_set();
        image_sub_ = nh_.subscribe(image_topic_name_, 1000, &Receiver_Message::image_callback, this);
        server_ = nh_.advertiseService(service_name_, &Receiver_Message::service_callback, this);
        pub_ = nh_.advertise<sensor_msgs::Image>("tsuchida_1", 1000);
    }

   

    void image_callback(const sensor_msgs::Image &mes)
    {
        // ROS_INFO_STREAM("tsuchida1");
        image_ = new sensor_msgs::Image(mes);
        pub_.publish(*image_);
        // ROS_INFO_STREAM("tsuchida2");
    }

    
    bool service_callback(denso_srvs::sensor_data_service::Request &request, denso_srvs::sensor_data_service::Response &response)
    {
        ROS_INFO_STREAM("kuru: " << request.call);
        response.image = *image_;
        return true;
    }

    void parameter_set()
    {
        pnh_.getParam("service_name", service_name_);
        pnh_.getParam("image_topic_name", image_topic_name_);
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    sensor_msgs::Image *image_;
    ros::ServiceServer server_;
    std::string camera_topic_name_, image_topic_name_, pointcloud_topic_name_;
    std::string service_name_;
    ros::NodeHandle pnh_;
    ros::Subscriber camera_sub_, image_sub_, pointcloud_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_receiver");
    ros::NodeHandle nh;
    Receiver_Message receive(nh);
    ros::spin();
}