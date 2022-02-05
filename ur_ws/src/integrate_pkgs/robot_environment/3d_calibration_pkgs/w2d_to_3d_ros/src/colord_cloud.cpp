#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>


template <typename T>
void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
{
    boost::shared_ptr<const T> message;
    message = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
    if (message != NULL) {
        final_message = *message;
    }
}

void tf_get(std::string target_frame, std::string source_frame, tf::StampedTransform &transform)
{
    ros::Time time = ros::Time(0);
    tf::TransformListener tf_listener_;
    try
    {
        tf_listener_.waitForTransform(target_frame, source_frame, time, ros::Duration(4.0));
        tf_listener_.lookupTransform(target_frame, source_frame, time, transform);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        ros::Duration(4.0).sleep();
    }
}

void operate(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, pcl::PointCloud<pcl::PointXYZ> trans_cloud, pcl::PointCloud<pcl::PointXYZRGB> &colored_point, cv::Mat &draw_image)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exeption %s", e.what());
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    ROS_INFO_STREAM("hayata: 1");
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_image = cv_image;
    ROS_INFO_STREAM("hayata: 2");

    image_geometry::PinholeCameraModel cam_model;
    ROS_INFO_STREAM("hayata: 3");

    cam_model.fromCameraInfo(cinfo);
    ROS_INFO_STREAM("cam_model tf: " << cam_model.tfFrame());

    colored_point.clear();
    int count = 0;
    int aida = 100;
    double debug_x = 0;
    double debug_y = 0;
    ROS_INFO_STREAM("trans size: " << trans_cloud.points.size());
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = trans_cloud.points.begin(); pt != trans_cloud.points.end(); ++pt)
    {
        count++;
        if (pt->z < 0)
        {
            ROS_INFO("passed point");
            continue;
        }
        cv::Point3d pt_cv(pt->x, pt->y, pt->z);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);
        // ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
        // ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
        debug_x += uv.x;
        debug_y += uv.y;
        // if (count % aida == 0) {
        //     ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
        //     ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
        // }
        double scale = 2;
        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            // ROS_INFO_STREAM("tuuka!!");
            // cv::Point2d converted_uv(uv.x + rgb_image.cols / 2, uv.y + rgb_image.rows / 2);
            // ROS_INFO_STREAM("x: " << pt->x << "  y: " << pt->y << "  z: " << pt->z);
            // ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
            // ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
            // cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(converted_uv.y, converted_uv.x);
            // cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(uv.y, uv.x);
            cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(uv.x, uv.y);
            cv::circle(draw_image, cv::Point(uv.x, uv.y), 1, cv::Scalar(0, 255, 0), 1, 1);
            
            cv::resize(draw_image, draw_image, draw_image.size(), 0.5, 0.5);
            pcl::PointXYZRGB buffer_point;
            ROS_INFO_STREAM("red: " << rgb[0] <<"  blue: " << rgb[1] << "   green: " << rgb[2]);
            double rgb_ave = (rgb[0] + rgb[1] + rgb[2]) / 3.0;
            // if (rgb_ave < upper_val) {
            //     continue;
            // }
            // if (rgb_ave > lower_val) {
            //     continue;
            // }
            buffer_point.x = pt->x;
            buffer_point.y = pt->y;
            buffer_point.z = pt->z;
            buffer_point.r = rgb[0];
            buffer_point.g = rgb[1];
            buffer_point.b = rgb[2];
            colored_point.push_back(buffer_point);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string cam_info, image_raw, input_pointcloud, output_pointcloud, target_frame, source_frame, cam_frame_id;
    double upper_val, lower_val;
    ros::Publisher pub;
    

    pnh.getParam("camera_info", cam_info);
    pnh.getParam("image_raw", image_raw);
    pnh.getParam("input_pointcloud", input_pointcloud);
    pnh.getParam("output_pointcloud", output_pointcloud);
    pnh.getParam("target_frame", target_frame);
    pnh.getParam("source_frame", source_frame);
    pnh.getParam("upper_filter_vals", upper_val);
    pnh.getParam("lower_filter_val", lower_val);
    pnh.getParam("camera_frame_id", cam_frame_id);

    pub = nh.advertise<sensor_msgs::PointCloud2>(output_pointcloud, 10);
    sensor_msgs::CameraInfo cinfo;
    sensor_msgs::Image image;
    sensor_msgs::PointCloud2 input_cloud, output_cloud;
    tf::StampedTransform transform;
    pcl::PointCloud<pcl::PointXYZ> cloud, trans_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> colored_pointcloud;
    

    get_one_message<sensor_msgs::CameraInfo>(cinfo, cam_info, nh, 10);
    get_one_message<sensor_msgs::Image>(image, image_raw, nh, 10);
    get_one_message<sensor_msgs::PointCloud2>(input_cloud, input_pointcloud, nh, 10);

    std::cout << "[" << cinfo.K[0] << " " << cinfo.K[1] << " " << cinfo.K[2] << "]" << std::endl;
    std::cout << "[" << cinfo.K[3] << " " << cinfo.K[4] << " " << cinfo.K[5] << "]" << std::endl;
    std::cout << "[" << cinfo.K[6] << " " << cinfo.K[7] << " " << cinfo.K[8] << "]" << std::endl;
    
    std::cout << image.height << " " << image.width << std::endl;
    

    // cinfo.header.frame_id = cam_frame_id;
    // ROS_INFO_STREAM(cinfo.header.frame_id);
    tf_get(target_frame, source_frame, transform);

    tf::Transform tf;
    tf.setOrigin(transform.getOrigin());
    tf.setRotation(transform.getRotation());
    ROS_INFO("transform_pointcloud11");
    // pcl::fromROSMsg(input_cloud, cloud);
    pcl::fromROSMsg(input_cloud, trans_cloud);
    // pcl_ros::transformPointCloud(cloud, trans_cloud, tf);
    
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exeption %s", e.what());
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    ROS_INFO_STREAM("hayata: 1");
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    cv::Mat draw_image = cv_image;
    ROS_INFO_STREAM("hayata: 2");

    image_geometry::PinholeCameraModel cam_model;
    ROS_INFO_STREAM("hayata: 3");

    cam_model.fromCameraInfo(cinfo);
    ROS_INFO_STREAM("cam_model tf: " << cam_model.tfFrame());

    colored_pointcloud.clear();
    int count = 0;
    int aida = 100;
    double debug_x = 0;
    double debug_y = 0;
    ROS_INFO_STREAM("trans size: " << trans_cloud.points.size());
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = trans_cloud.points.begin(); pt != trans_cloud.points.end(); ++pt)
    {
        count++;
        if (pt->z < 0)
        {
            ROS_INFO("passed point");
            continue;
        }
        cv::Point3d pt_cv(pt->x, pt->y, pt->z);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);
        // ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
        // ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
        debug_x += uv.x;
        debug_y += uv.y;
        // if (count % aida == 0) {
        //     ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
        //     ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
        // }
        if (uv.x > (-rgb_image.cols / 1.5) && uv.x < (rgb_image.cols / 1.5) && uv.y > (-rgb_image.rows / 1.5) &&
        uv.y < (rgb_image.rows / 1.5))
        {
            // ROS_INFO_STREAM("tuuka!!");
            cv::Point2d converted_uv(uv.x + rgb_image.cols / 2, uv.y + rgb_image.rows / 2);
            // ROS_INFO_STREAM("x: " << pt->x << "  y: " << pt->y << "  z: " << pt->z);
            // ROS_INFO_STREAM("uv.x: " << uv.x << "  uv.y: " << uv.y);
            // ROS_INFO_STREAM("image_col / 2: " << rgb_image.cols / 2 << "   image_rows / 2: " << rgb_image.rows / 2);
            cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(converted_uv.y, converted_uv.x);
            cv::circle(draw_image, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 255, 0), 1, 1);
            pcl::PointXYZRGB buffer_point;
            // ROS_INFO_STREAM("red: " << rgb[0] <<"  blue: " << rgb[1] << "   green: " << rgb[2]);
            double rgb_ave = (rgb[0] + rgb[1] + rgb[2]) / 3.0;
            // if (rgb_ave < upper_val) {
            //     continue;
            // }
            // if (rgb_ave > lower_val) {
            //     continue;
            // }
            buffer_point.x = pt->x;
            buffer_point.y = pt->y;
            buffer_point.z = pt->z;
            buffer_point.r = rgb[0];
            buffer_point.g = rgb[1];
            buffer_point.b = rgb[2];
            colored_pointcloud.push_back(buffer_point);
        }
    }
    ros::Rate loop(10);
    while (ros::ok())
    {
        // ROS_INFO_STREAM("color size: " << colored_pointcloud.points.size());
        // get_one_message<sensor_msgs::CameraInfo>(cinfo, cam_info, nh, 10);
        // std::cout << "[" << cinfo.K[0] << " " << cinfo.K[1] << " " << cinfo.K[2] << "]" << std::endl;
        // std::cout << "[" << cinfo.K[3] << " " << cinfo.K[4] << " " << cinfo.K[5] << "]" << std::endl;
        // std::cout << "[" << cinfo.K[6] << " " << cinfo.K[7] << " " << cinfo.K[8] << "]" << std::endl;
        // std::cout << cinfo.header.frame_id << std::endl;
        sensor_msgs::CameraInfo cinfo;
        sensor_msgs::Image image;
        get_one_message<sensor_msgs::CameraInfo>(cinfo, cam_info, nh, 10);
        get_one_message<sensor_msgs::Image>(image, image_raw, nh, 10);
        get_one_message<sensor_msgs::PointCloud2>(input_cloud, input_pointcloud, nh, 10);
        pcl::fromROSMsg(input_cloud, trans_cloud);
        operate(cinfo, image, trans_cloud, colored_pointcloud, draw_image);
        pcl::toROSMsg(colored_pointcloud, output_cloud);
        output_cloud.header.frame_id = "photoneo_center_optical_frame";
    
        cv::imshow("fe", draw_image);
        cv::waitKey(10);
        pub.publish(output_cloud);
        loop.sleep();
        
    }

    
    

    return 0;
}