#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <denso_srvs/sensor_data_service.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <denso_msgs/denso_pointcloud2.h>

#include <all_estimator/crop_cloud_server.h>

class Sensor_Data_Service
{
public:
    Sensor_Data_Service(ros::NodeHandle nh):
    pnh_("~"),
    nh_(nh)
    {
        get_topic_name();
        server_ = nh_.advertiseService(service_name, &Sensor_Data_Service::service_callback, this);
        camera_sub_ = nh_.subscribe(camera_topic, 10, &Sensor_Data_Service::camera_callback, this);
        image_sub_ = nh_.subscribe(image_topic, 10, &Sensor_Data_Service::image_callback, this);
        point_cloud2_sub_ = nh_.subscribe(point_cloud2_topic, 10, &Sensor_Data_Service::point_cloud2_callback, this);
        pub = nh_.advertise<sensor_msgs::PointCloud2>("chatter", 10);
        pub_1_ = nh_.advertise<sensor_msgs::PointCloud2>("moto_cloud", 10);
        pub_2_ = nh_.advertise<sensor_msgs::PointCloud2>("crop_cloud", 10);
        pub_3_ = nh_.advertise<sensor_msgs::PointCloud2>("filter_cloud", 10);
    }

    void get_topic_name()
    {
        pnh_.getParam("image_topic", image_topic);
        pnh_.getParam("camera_info_topic", camera_topic);
        pnh_.getParam("service_name", service_name);
        pnh_.getParam("point_cloud2_topic", point_cloud2_topic);

        pnh_.getParam("crop_x_min", min.x);
        pnh_.getParam("crop_y_min", min.y);
        pnh_.getParam("crop_z_min", min.z);
        pnh_.getParam("crop_x_max", max.x);
        pnh_.getParam("crop_y_max", max.y);
        pnh_.getParam("crop_z_max", max.z);
        pnh_.getParam("world_frame", world_frame_);
    }

    bool service_callback(denso_srvs::sensor_data_service::Request &req, denso_srvs::sensor_data_service::Response &response)
    {
        response.image = image_;
        response.camera_info = camera_;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud, cloud_without, cloud_filtered_pcl;
        ROS_INFO_STREAM(point_cloud2_.header.frame_id);
        
        while (true)
        {
            try
            {
                listener_.lookupTransform(world_frame_, point_cloud2_.header.frame_id, ros::Time(0), transform_);
                ROS_INFO_ONCE("I got a transfomr");
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        sensor_msgs::PointCloud2 msg_transformed;
        pcl_ros::transformPointCloud(world_frame_, transform_, point_cloud2_, msg_transformed);
        pub_1_.publish(msg_transformed);

        pcl::fromROSMsg(msg_transformed, pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>(pcl_cloud));

        Eigen::Vector4f minPoint;
        minPoint[0] = min.x;  // define minimum point x
        minPoint[1] = min.y;  // define minimum point y
        minPoint[2] = min.z;  // define minimum point z

        Eigen::Vector4f maxPoint;
        maxPoint[0] = max.x;  // define max point x
        maxPoint[1] = max.y;  // define max point y
        maxPoint[2] = max.z;  // define max point z

        Eigen::Vector3f boxTranslatation;
        boxTranslatation[0] = 0;
        boxTranslatation[1] = 0;
        boxTranslatation[2] = 0;

        Eigen::Vector3f boxRotation;
        boxRotation[0] = 0;  // rotation around x-axis
        boxRotation[1] = 0;  // rotation around y-axis
        boxRotation[2] = 0;  // in radians rotation around z-axis. this rotates your

        pcl::CropBox<pcl::PointXYZ> cropFilter;
        cropFilter.setInputCloud(cloud_crop);
        cropFilter.setMin(minPoint);
        cropFilter.setMax(maxPoint);
        cropFilter.setTranslation(boxTranslatation);
        cropFilter.setRotation(boxRotation);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        cropFilter.filter(*cloud_filtered);

        pcl::PointCloud<pcl::PointXYZ> crop_box;
        crop_box = *cloud_filtered;
        
        sensor_msgs::PointCloud2 cloud_kasika;
        pcl::toROSMsg(crop_box,cloud_kasika);
        pub_2_.publish(cloud_kasika);
        // ROS_INFO_STREAM("chatter: " << cloud_kasika.header.frame_id);
        // pub.publish(cloud_kasika);

        cloud_filtered_pcl = filtering(crop_box);

        sensor_msgs::PointCloud2 cloud_kasika_2;
        pcl::toROSMsg(cloud_filtered_pcl,cloud_kasika_2);
        pub_3_.publish(cloud_kasika_2);

        segment(inliers, cloud_filtered_pcl);
        cloud_without = extract(inliers, cloud_filtered_pcl);

        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(cloud_without, cloud);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_pcl(new pcl::PointCloud<pcl::PointXYZ>(cloud_filtered_pcl));
        // *cloud_without_pcl = cloud_without;

         while (true)
        {
            try
            {
                listener_.lookupTransform(point_cloud2_.header.frame_id, world_frame_,  ros::Time(0), transform_);
                ROS_INFO_ONCE("I got a transfomr");
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        sensor_msgs::PointCloud2 last_transformed;
        pcl_ros::transformPointCloud(point_cloud2_.header.frame_id , transform_, cloud, last_transformed);
        pcl::PointCloud<pcl::PointXYZ> cloud_without_segmented_pcl;
        pcl::fromROSMsg(last_transformed, cloud_without_segmented_pcl);
        last_transformed.header.frame_id = point_cloud2_.header.frame_id;
        pub.publish(last_transformed);

        // ROS_INFO_STREAM("size2: " << cloud_filtered->size());
        denso_msgs::denso_pointcloud2 denso_cloud2;
        for(size_t i = 0; i < cloud_without_segmented_pcl.points.size(); i++){
            denso_cloud2.x.push_back(cloud_without_segmented_pcl.points[i].x);
            denso_cloud2.y.push_back(cloud_without_segmented_pcl.points[i].y);
            denso_cloud2.z.push_back(cloud_without_segmented_pcl.points[i].z);
        }


        std::cout <<cloud_without_segmented_pcl.points.size() << std::endl;
        std::cout <<denso_cloud2.x.size() << std::endl;
        response.point_cloud2 = denso_cloud2;
        
        return true;
    }


    void image_callback(const sensor_msgs::ImageConstPtr msg)
    {
        image_ = *msg;
    }

    void camera_callback(const sensor_msgs::CameraInfoConstPtr msg)
    {
        camera_ = *msg;
    }

    void point_cloud2_callback(const sensor_msgs::PointCloud2ConstPtr msg)
    {
        point_cloud2_ = *msg;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string image_topic, camera_topic, point_cloud2_topic, service_name;
    ros::ServiceServer server_;
    ros::Subscriber image_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber point_cloud2_sub_;
    ros::Publisher pub, pub_1_, pub_2_, pub_3_;
    pcl::PointXYZ min, max;
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    std::string world_frame_;

    void segment(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
    {
        pcl::ModelCoefficients coefficients_pcl;
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        // Create the segmentation object
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        // segmentation.setDistanceThreshold(0.005); //閾値
        segmentation.setDistanceThreshold(0.0008); //L515
        segmentation.setInputCloud(pcl_cloud.makeShared());
        segmentation.segment(*inliers, coefficients_pcl);
    }

    pcl::PointCloud<pcl::PointXYZ> extract(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_without;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        extract.setInputCloud(pcl_cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(cloud_without);
        return cloud_without;
    }

    pcl::PointCloud<pcl::PointXYZ> filtering(pcl::PointCloud<pcl::PointXYZ>&pcl_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(pcl_cloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered_pcl);
        return cloud_filtered_pcl;
    }


protected:
    sensor_msgs::Image image_;
    sensor_msgs::CameraInfo camera_;
    sensor_msgs::PointCloud2 point_cloud2_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_server");
    ros::NodeHandle nh;
    Sensor_Data_Service sen(nh);
    ros::spin();
}
