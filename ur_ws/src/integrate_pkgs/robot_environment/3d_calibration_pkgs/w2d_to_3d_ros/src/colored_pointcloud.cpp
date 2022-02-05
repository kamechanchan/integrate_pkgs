#include <w2d_to_3d_ros/colored_pointcloud.hpp>
#include <mutex>

std::mutex m;

Colored_PointCloud::Colored_PointCloud(ros::NodeHandle nh)
    : nh_(nh)
{
    pnh_ = new ros::NodeHandle("~");
    parameter_set();
}

void Colored_PointCloud::parameter_set()
{
    pnh_->getParam("camera_topic_name", camera_topic_name_);
    pnh_->getParam("image_topic_name", image_topic_name_);
    pnh_->getParam("input_cloud_topic_name", inputcloud_topic_name_);
    pnh_->getParam("output_cloud_topic_name", outputcloud_topic_name_);
    pnh_->getParam("target_frame", target_frame_);
    pnh_->getParam("source_frame", source_frame_);
    pnh_->getParam("paramter_output_file_path", paramter_output_file_path);
    pnh_->getParam("f_scale", f_scale_);
    pnh_->getParam("cx_scale", cx_scale_);
    pnh_->getParam("cy_scale", cy_scale_);
    ofs = new std::ofstream(paramter_output_file_path);
    ROS_INFO_STREAM(camera_topic_name_);
    ROS_INFO_STREAM(image_topic_name_);
    ROS_INFO_STREAM(inputcloud_topic_name_);
    output_pub = nh_.advertise<sensor_msgs::PointCloud2>(outputcloud_topic_name_, 10);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    // sensor_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, inputcloud_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_Sub_Type>(Sync_Sub_Type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&Colored_PointCloud::InputCallback, this, _1, _2));
}

void Colored_PointCloud::InputCallback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
{
    std::lock_guard<std::mutex> lock(m);
    sensor_msgs::CameraInfo cinfo = *cam_msgs;
    sensor_msgs::Image image = *image_msgs;
    pcl::PointCloud<pcl::PointXYZ> trans_cloud;
    sensor_msgs::PointCloud2 cloud_msgs;
    get_one_message(cloud_msgs, inputcloud_topic_name_, nh_, 10);
    pcl::fromROSMsg(cloud_msgs, trans_cloud);
    std::cout << "[" << cinfo.K[0] << " " << cinfo.K[1] << " " << cinfo.K[2] << "]" << std::endl;
    std::cout << "[" << cinfo.K[3] << " " << cinfo.K[4] << " " << cinfo.K[5] << "]" << std::endl;
    std::cout << "[" << cinfo.K[6] << " " << cinfo.K[7] << " " << cinfo.K[8] << "]" << std::endl;
    colored_get(cinfo, image, trans_cloud, colored_pointcloud_, draw_image_);
    pcl::toROSMsg(colored_pointcloud_, output_cloud_);
    output_cloud_.header.frame_id = cloud_msgs.header.frame_id;
    cv::resize(draw_image_, draw_image_, cv::Size(), 0.4, 0.4);
    cv::imshow("draw", draw_image_);
    cv::imwrite("/home/ericlab/haji.jpg", draw_image_);
    cv::waitKey(10);
    output_pub.publish(output_cloud_);
    
}

void Colored_PointCloud::colored_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, pcl::PointCloud<pcl::PointXYZ> trans_cloud, pcl::PointCloud<pcl::PointXYZRGB> &colored_point, cv::Mat &draw_image)
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
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_image = cv_image;

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
        // uv = cam_model.project3dToPixel(pt_cv);
        uv = project3d_to_pixel(pt_cv, cinfo);
        
        debug_x += uv.x;
        debug_y += uv.y;
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            cv::Vec3d rgb = rgb_image.at<cv::Vec3b>(uv.y, uv.x);
            cv::circle(draw_image, cv::Point(uv.x, uv.y), 1, cv::Scalar(0, 0, 255), 1, 1);
            
            cv::resize(draw_image, draw_image, draw_image.size(), 0.25, 0.25);
            pcl::PointXYZRGB buffer_point;
            double rgb_ave = (rgb[0] + rgb[1] + rgb[2]) / 3.0;
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
 cv::Point2d Colored_PointCloud::project3d_to_pixel(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg) 
{
    cv::Point2d uv_rect;
    fx = static_cast<double>(cinfo_msg.K[0]) * f_scale_;
    tx = static_cast<double>(cinfo_msg.K[1]);
    cx = static_cast<double>(cinfo_msg.K[2]) * cx_scale_;
    fy = static_cast<double>(cinfo_msg.K[4]) * f_scale_;
    ty = static_cast<double>(cinfo_msg.K[3]);
    cy = static_cast<double>(cinfo_msg.K[5]) * cy_scale_;
    uv_rect.x = (fx *xyz.x + tx) / xyz.z + cx;
    uv_rect.y = (fy * xyz.y + ty) / xyz.z + cy;
    return uv_rect;
}