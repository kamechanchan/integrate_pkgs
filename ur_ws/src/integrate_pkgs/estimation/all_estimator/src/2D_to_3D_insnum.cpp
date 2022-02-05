
#include <all_estimator/2D_to_3D_insnum.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <iostream>


YoloBridge::YoloBridge(ros::NodeHandle &nh) :
    nh_(nh),
    radious_(0.05),
    pnh_("~"),
    buffer_(),
    lister_(buffer_)
{
    parameter_set();
}



void YoloBridge::callback(sensor_msgs::CameraInfoConstPtr cam_msgs)
{
    cinfo_ = *cam_msgs;
    ROS_INFO_STREAM("start" << "kuruze");
    // server_ = nh_.advertiseService("input_bridge_san", &YoloBridge::bridge, this);
}

bool YoloBridge::bridge(denso_srvs::input_bridge::Request &req, denso_srvs::input_bridge::Response &res){
    srv_.response.x.clear();
    srv_.response.y.clear();
    srv_.response.z.clear();
    srv_.response.instances.clear();
    // get_one_message<denso_msgs::yolo_bridge>(req, YOLO_topic_name_, nh_, timespan_);
    ROS_INFO_STREAM("iijanai");
    get_one_message(cloud_msgs_, inputcloud_topic_name_, nh_, timespan_);
    // ROS_INFO_STREAM("iijanai1" << cloud_msgs_);
    pcl::fromROSMsg(cloud_msgs_, trans_cloud_);
    // ROS_INFO_STREAM("iijanai2");
    // ROS_INFO_STREAM(req.out_data[1]);
    // ROS_INFO_STREAM("koreka: " << size(req.output_img));
    cv_bridge::CvImageConstPtr cv_img_ptr;
    cv_img_ptr = cv_bridge::toCvCopy(req.output_img, sensor_msgs::image_encodings::TYPE_8UC3);
    // ROS_INFO_STREAM("iijanai3");
    for (int i = 0; i < cv_img_ptr->image.rows; i++) {
        std::vector<int> iim;
        for (int j = 0; j < cv_img_ptr->image.cols; j++) {
            iim.push_back(0);
        }
        image_instance_.push_back(iim);
    }
    // ROS_INFO_STREAM("iijanai4");
    cv::Mat draw_img_(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    draw_img_ = cv_img_ptr->image;
    // float a;
    cv::Point2d points_1;
    cv::Point2d points_2;
    std::vector<std::vector<cv::Point2d>> uv_points_;
    // ROS_INFO_STREAM("iijanai5");
    for (int i=0; i<req.out_data.size(); i++){
        std::vector<cv::Point2d> init_1;
        points_1.x = req.out_data[i].data[0];
        points_1.y = req.out_data[i].data[1];
        points_2.x = req.out_data[i].data[2];
        points_2.y = req.out_data[i].data[3];
        init_1.push_back(points_1);
        init_1.push_back(points_2);
        uv_points_.push_back(init_1);
        // ROS_INFO_STREAM("tanomuze: " << i);
    }
    ROS_INFO_STREAM("bounding_box_num" << uv_points_.size());


    image_instance_ = write_instance(uv_points_, draw_img_);
    ros::WallTime waka = ros::WallTime::now();
    hurui(trans_cloud_, image_instance_, req.input_img, cinfo_, color_cloud_);
    ros::WallTime tte = ros::WallTime::now();
    ros::WallDuration kure = waka - tte;
    ROS_INFO_STREAM("hurui: " << kure.toSec());
    pcl::toROSMsg(color_cloud_, output_cloud_msgs_);
    color_cloud_.clear();
    ROS_INFO_STREAM("iijanai3");
    
    output_cloud_msgs_.header.frame_id = cloud_msgs_.header.frame_id;
    std::cout << output_cloud_msgs_.header.frame_id << std::endl;
    output_pub_.publish(output_cloud_msgs_);
    ROS_INFO_STREAM("muryokusyo");

    res.x = srv_.response.x;
    res.y = srv_.response.y;
    res.z = srv_.response.z;
    res.instances = srv_.response.instances;
    ROS_INFO_STREAM("owatta");

    return true;
}

void YoloBridge::parameter_set()
{
    pnh_.getParam("camera_topic_name", camera_topic_name_);
    pnh_.getParam("f_scale", f_scale_);
    pnh_.getParam("cx_scale", cx_scale_);
    pnh_.getParam("cy_scale", cy_scale_);
    pnh_.getParam("radious", radious_);
    pnh_.getParam("model_name", model_name_);
    pnh_.getParam("the_number_of_data", the_number_of_data);
    pnh_.getParam("YOLO_topic_name", YOLO_topic_name_);
    pnh_.getParam("inputcloud_topic_name", inputcloud_topic_name_);
    pnh_.getParam("output_topic_name", output_topic_name_);
    pnh_.getParam("timespan", timespan_);
    pnh_.getParam("instance_number", instance_number_);
    output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 10);
    sub_ = nh_.subscribe(camera_topic_name_, 10, &YoloBridge::callback, this);
    server_ = nh_.advertiseService("input_bridge", &YoloBridge::bridge, this);
}



std::vector<std::vector<int>> YoloBridge::write_instance(std::vector<std::vector<cv::Point2d>> point_2d, cv::Mat img_size_img)
{
    std::vector<std::vector<int>> instance_img_sasyo;
    for (int i = 0; i < img_size_img.rows; i++) {
        std::vector<int> iim;
        for (int j = 0; j < img_size_img.cols; j++) {
            iim.push_back(0);
        }
        instance_img_sasyo.push_back(iim);
    }
    for (int i = 0; i < point_2d.size(); i++) {
        int x1 = static_cast<int>(point_2d[i][0].x);
        int x2 = static_cast<int>(point_2d[i][1].x);
        int y1 = static_cast<int>(point_2d[i][0].y);
        int y2 = static_cast<int>(point_2d[i][1].y);
        // ROS_INFO_STREAM("naruhodo" << i);
        if (x1 > x2) {
            swap(x1, x2);
        }
        if (y1 > y2) {
            swap(y1, y2);
        }
        int count = 0;
        for (int k = y1; k <= y2; k++) 
        {
            for (int l = x1; l <= x2; l++) {
                instance_img_sasyo[k][l] = i+1;
            }
        }

    }
    return instance_img_sasyo;
}

void YoloBridge::hurui(pcl::PointCloud<pcl::PointXYZ> input_pcl_cloud, std::vector<std::vector<int>> instance, sensor_msgs::Image image, sensor_msgs::CameraInfo cinfo, pcl::PointCloud<pcl::PointXYZ> &outcloud_pcl_cloud)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        // cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exeption %s", e.what());
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    int coun = 0;
    // srv_ = denso_srvs::bounding();
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = input_pcl_cloud.points.begin(); pt != input_pcl_cloud.points.end(); ++pt)
    {
        // coun++;
        // ROS_INFO_STREAM(coun);
        ros::WallTime waka = ros::WallTime::now();
        
        if (pt->z < 0)
        {
            ROS_INFO_STREAM("pass point");
            continue;
        }
        cv::Point3d pt_cv(pt->x, pt->y, pt->z);
        cv::Point2d uv;
        uv = project3d_to_pixel_origin(pt_cv, cinfo);
       // cv::Mat rgb_image;
        //cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
        // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.y < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale)
            && uv.y < (rgb_image.rows / scale))
        {
            
            pcl::PointXYZ buffer_point;
            int x = static_cast<int>(uv.x);
            int y = static_cast<int>(uv.y);

            if (instance[y][x] == instance_number_) { //ここの数がどのバウンディングボックスをパブリッシュするか決定
                cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(y, x);
                buffer_point.x = pt->x;
                buffer_point.y = pt->y;
                buffer_point.z = pt->z;
                srv_.response.x.push_back(pt->x);
                srv_.response.y.push_back(pt->y);
                srv_.response.z.push_back(pt->z);

                outcloud_pcl_cloud.push_back(buffer_point);
            }
            // else {
            //     cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(y, x);
            //     buffer_point.x = pt->x;
            //     buffer_point.y = pt->y;
            //     buffer_point.z = pt->z;
            //     srv_.response.x.push_back(pt->x);
            //     srv_.response.y.push_back(pt->y);
            //     srv_.response.z.push_back(pt->z);

            //     outcloud_pcl_cloud.push_back(buffer_point);
            // }
        }
    }
    for (int i = 0; i < 20; i+=2) {
        std::cout << "requset_x: " << srv_.response.x[i] << "  z: " << srv_.response.z[i] <<  std::endl;
    }
    
    // temporary instance data
    srv_.response.instances = {1, 2};

    // client_ = nh_.serviceClient<denso_srvs::bounding>("bounding_box");
  
    // auto segment_data_ = client_.call(srv_);
    // ROS_INFO_STREAM("tanomuze");
    // ROS_INFO_STREAM(outcloud_pcl_cloud.size());
}

cv::Point2d YoloBridge::project3d_to_pixel_origin(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    cv::Point2d uv_rect;
    fx_ = cinfo_msg.K[0] * f_scale_;
    tx_ = cinfo_msg.K[1];
    cx_ = cinfo_msg.K[2] * cx_scale_;
    fy_ = cinfo_msg.K[4] * f_scale_;
    ty_ = cinfo_msg.K[3];
    cy_ = cinfo_msg.K[5] * cy_scale_;
    uv_rect.x = (fx_ * xyz.x + tx_) / xyz.z + cx_;
    uv_rect.y = (fy_ * xyz.y + ty_) / xyz.z + cy_;
    return uv_rect;
}

int main(int argc, char** argv)
{
    // std::cout << "iaipa";
    // ros::Node
    ros::init(argc, argv, "tuschda_inti");
    ros::NodeHandle nh;
    YoloBridge Ishiyama(nh);
    ros::spin();
}
