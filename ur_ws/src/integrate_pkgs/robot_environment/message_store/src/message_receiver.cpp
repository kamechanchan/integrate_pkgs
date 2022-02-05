#include <ros/ros.h>
#include <denso_srvs/sensor_data_service.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <denso_msgs/out_segmentation.h>


class Receiver_Message
{
public:
    Receiver_Message(ros::NodeHandle &nh) : pnh_("~"), nh_(nh)
    {
        parameter_set();
        camera_sub_ = nh_.subscribe(camera_topic_name_, 10, &Receiver_Message::camera_callback, this);
        image_sub_ = nh_.subscribe(image_topic_name_, 10, &Receiver_Message::camera_callback, this);
        pointcloud_sub_ = nh_.subscribe(pointcloud_topic_name_, 10, &Receiver_Message::camera_callback, this);
        server_ = nh_.advertiseService(service_name_, &Receiver_Message::service_callback, this);
    }

    void camera_callback(const sensor_msgs::CameraInfo &mes)
    {
        *cinfo_ = mes;
    }

    void image_callback(const sensor_msgs::Image &mes)
    {
        *image_ = mes;
    }

    void pointcloud_callback(const sensor_msgs::PointCloud2 &mes)
    {
        *pointcloud_ = mes;
    }

    bool service_callback(denso_srvs::sensor_data_service::Request &request, denso_srvs::sensor_data_service::Response &response)
    {
        response.camera_info = *cinfo_;
        response.image = *image_;
        pcl::PointCloud<pcl::PointXYZ> pcd_cloud_;
        denso_msgs::out_segmentation out_;
        pcl::fromROSMsg(*pointcloud_, pcd_cloud_);
        for (int i = 0; i < pcd_cloud_.points.size(); i++) {
            out_.x.push_back(pcd_cloud_.points[i].x);
            out_.y.push_back(pcd_cloud_.points[i].y);
            out_.z.push_back(pcd_cloud_.points[i].z);
        }
        response.out_point = out_;
        return true;
    }

    void parameter_set()
    {
        pnh_.getParam("camera_topic_name", camera_topic_name_);
        pnh_.getParam("image_topic_name", image_topic_name_);
        pnh_.getParam("pointcloud_topic_name", pointcloud_topic_name_);
    }
private:
    ros::NodeHandle nh_;
    sensor_msgs::CameraInfo *cinfo_;
    sensor_msgs::Image *image_;
    sensor_msgs::PointCloud2 *pointcloud_;
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