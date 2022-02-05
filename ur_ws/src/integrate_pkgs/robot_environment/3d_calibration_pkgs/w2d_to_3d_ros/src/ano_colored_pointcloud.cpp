#include <w2d_to_3d_ros/ano_colored_pointcloud.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

#define PI 3.141592


std::mutex m;
Annotation_yolo::Annotation_yolo(ros::NodeHandle &nh) :
    nh_(nh),
    radious_(0.05),
    save_count_(0),
    work_count_(0),
    pnh_("~")
{
    buffer_ = new tf2_ros::Buffer();
    lister_ = new tf2_ros::TransformListener(*buffer_);
    parameter_set();
    
    
    
    // ROS_INFO_STREAM("wataru");
}

void Annotation_yolo::parameter_set()
{
    pnh_.getParam("camera_topic_name", camera_topic_name_);
    pnh_.getParam("image_topic_name", image_topic_name_);
    pnh_.getParam("source_frame", source_frame_);
    pnh_.getParam("target_frame", target_frame_);
    pnh_.getParam("f_scale", f_scale_);
    pnh_.getParam("cx_scale", cx_scale_);
    pnh_.getParam("cy_scale", cy_scale_);
    pnh_.getParam("radious", radious_);
    pnh_.getParam("image_dir_name", image_dir_name_);
    pnh_.getParam("filebasename", filebasename_);
    pnh_.getParam("work_count", work_count_);
    pnh_.getParam("model_name", model_name_);
    pnh_.getParam("world_frame", world_frame_);
    pnh_.getParam("label_dir_name", label_dir_name_);
    pnh_.getParam("boxes_dir_name", boxes_dir_name_);
    pnh_.getParam("the_number_of_data", the_number_of_data);
    paramter_set_bara(model_name_, work_count_);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_topic_name_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic_name_, 10);
    sensor_sync_ = new message_filters::Synchronizer<Sync_Sub_type>(Sync_Sub_type(10), *camera_sub_, *image_sub_);
    sensor_sync_->registerCallback(boost::bind(&Annotation_yolo::InputCallback, this, _1, _2));
}

void Annotation_yolo::InputCallback(sensor_msgs::CameraInfoConstPtr cam_msgs, sensor_msgs::ImageConstPtr image_msgs)
{
    if (save_count_ == the_number_of_data) {
        ROS_INFO_STREAM("the number of data: " << the_number_of_data);
        nh_.setParam("not_finish", false);
    }
    std::lock_guard<std::mutex> lock(m);
    sensor_msgs::CameraInfo cinfo = *cam_msgs;
    sensor_msgs::Image image1 = *image_msgs;
    // geometry_msgs::TransformStamped transform;
    // tf_get(source_frame_, target_frame_, transform);
    // box_get(cinfo, image1, transform, draw_image_);
    geometry_msgs::TransformStamped trans_source;
    // tf2::Quaternion q1;
    // tf2::convert(trans_source.transform.rotation, q1);
    
    tf_get(world_frame_, source_frame_, trans_source);
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int i = 0; i < work_count_; i++) {
        geometry_msgs::TransformStamped trans;
        tf_get(world_frame_, target_frames_[i], trans);
        transforms.push_back(trans);
    }
    for (int i = 0; i < transforms.size(); i++) {
        geometry_msgs::Vector3 trans = transforms[i].transform.translation;
        geometry_msgs::Quaternion quat = transforms[i].transform.rotation;
        // ROS_INFO_STREAM("translation:   x: " << trans.x << "   y: " << trans.y << " z: " << trans.z);
        // ROS_INFO_STREAM("rotation:      x: " << quat.x << "  y: " << quat.y << " z: " << quat.z << " w: " << quat.w);
    }
    // box_get(cinfo, image1, transforms, draw_image_, work_count_);
    std::vector<cv::Point3d> point_3d;
    rotation_convert(trans_source, transforms, point_3d);
    std::vector<std::vector<cv::Point2d>> uv_points;
    box_get(cinfo, image1, point_3d, draw_image_, uv_points);
    // box_get(cinfo, image1, point_3d, draw_image_, work_count_);
    cv::resize(draw_image_, draw_image_, cv::Size(), 0.7, 0.7);
    cv::imshow("windoue", draw_image_);
    cv::waitKey(10);
    nh_.getParam("write_is_ok", write_is_ok_);
    // ROS_INFO_STREAM("write_is_osk: " << bool(write_is_ok_));
    
    if (write_is_ok_ && save_count_ < the_number_of_data) {
        ROS_INFO_STREAM("process rate: " << save_count_ + 1 << "/" << the_number_of_data);
        nh_.setParam("write_is_ok", false);
        std::string img_path = image_dir_name_ + "/" + filebasename_ + "_" + std::to_string(save_count_) + ".jpg";
        cv::Mat origin_img;
        get_original_image(image1, origin_img);
        cv::imwrite(img_path, origin_img);
        std::string box_path = boxes_dir_name_ + "/" + filebasename_ + "_" + std::to_string(save_count_) + ".jpg";
        cv::imwrite(box_path, draw_image_);
        std::string label_path = label_dir_name_ + "/" + filebasename_ + "_" + std::to_string(save_count_) + ".txt";
        std::ofstream file(label_path);
        for (int i = 0; i < uv_points.size(); i++) {
            file << model_name_ << " " << uv_points[i][0].x << " " << uv_points[i][0].y << " " << uv_points[i][1].x << " " << uv_points[i][1].y << std::endl;
        }
        file.close();
        save_count_++;
        nh_.setParam("move_is_ok", true);
    }
}

void Annotation_yolo::tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &trans)
{
    try
    {
        trans = buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
        
        
        ROS_INFO_ONCE("I got a transfomr");
    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        ros::Duration(1.0).sleep();
        return;
    }
    
    
}

void Annotation_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, geometry_msgs::TransformStamped trans, cv::Mat &draw_img)
{
    // ROS_INFO_STREAM("hatayu");
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_img = cv_image;
    // ROS_INFO_STREAM("x: " << trans.transform.translation.x);
    // ROS_INFO_STREAM("y: " << trans.transform.translation.y);
    // ROS_INFO_STREAM("z: " << trans.transform.translation.z);
    float x = trans.transform.translation.x;
    float y = trans.transform.translation.y;
    float z = trans.transform.translation.z;
    
    cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
    cv::Point2d uv, uv_x1, uv_x2;
    uv = project3d_to_pixel(pt_cv, cinfo);
    uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
    uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
    // ROS_INFO_STREAM("uv.x: " << uv.x);
    // ROS_INFO_STREAM("uv.y: " << uv.y);
    double scale = 1;
    if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
    uv.y < (rgb_image.rows / scale))
    {
        cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(uv.y, uv.x);
        // cv::rotate(draw_img, draw_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
        cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x1.y), cv::Point(uv_x2.x, uv_x2.y), cv::Scalar(255, 0, 0), 4);
    }
}

void Annotation_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, std::vector<geometry_msgs::TransformStamped> trans_s, cv::Mat &draw_img, int work_count)
{
    // ROS_INFO_STREAM("hatayu");
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_img = cv_image;
    
    int count = 0;
    int aida = 100;

    for (int i = 0; i < work_count; i++) {
        float x = trans_s[i].transform.translation.y;
        float y = trans_s[i].transform.translation.x;
        float z = trans_s[i].transform.translation.z;
        cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
        cv::Point2d uv, uv_x1, uv_x2;
        uv = project3d_to_pixel(pt_cv, cinfo);
        uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
        uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(uv.y, uv.x);
            // ROS_INFO_STREAM("image_" << i);
            // cv::rotate(draw_img, draw_img, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
            cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x1.y), cv::Point(uv_x2.x, uv_x2.y), cv::Scalar(255, 0, 0), 4);
        }
    }
}

void Annotation_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, std::vector<cv::Point3d> trans_s, cv::Mat &draw_img, int work_count)
{
    // ROS_INFO_STREAM("hatayu");
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_img = cv_image;
    
    int count = 0;
    int aida = 100;

    for (int i = 0; i < work_count; i++) {
        double x = trans_s[i].x;
        double y = trans_s[i].y;
        double z = trans_s[i].z;
        cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
        cv::Point2d uv, uv_x1, uv_x2;
        uv = project3d_to_pixel(pt_cv, cinfo);
        uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
        uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
        double scale = 1;
        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
            cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x1.y), cv::Point(uv_x2.x, uv_x2.y), cv::Scalar(0, 255, 0), 4);
        }

    }
}

void Annotation_yolo::box_get(sensor_msgs::CameraInfo cinfo, sensor_msgs::Image image, std::vector<cv::Point3d> trans_s, cv::Mat &draw_img, std::vector<std::vector<cv::Point2d>> &uv_points)
{
    // ROS_INFO_STREAM("hatayu");
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_img_ptr->image;
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);
    draw_img = cv_image;
    
    int count = 0;
    int aida = 100;

    for (int i = 0; i < trans_s.size(); i++) {
        double x = trans_s[i].x;
        double y = trans_s[i].y;
        double z = trans_s[i].z;
        cv::Point3d pt_cv(y, x, z), pt_cv_x1(y - radious_, x - radious_, z), pt_cv_x2(y + radious_, x + radious_, z);
        cv::Point2d uv, uv_x1, uv_x2;
        uv = project3d_to_pixel(pt_cv, cinfo);
        uv_x1 = project3d_to_pixel(pt_cv_x1, cinfo);
        uv_x2 = project3d_to_pixel(pt_cv_x2, cinfo);
        double scale = 1;
        // ROS_INFO_STREAM(uv.x << "  " << uv.y << "   imgasize" << rgb_image.cols << "  " << rgb_image.rows);

        if (uv.x > (-rgb_image.cols / scale) && uv.x < (rgb_image.cols / scale) && uv.y > (-rgb_image.rows / scale) &&
        uv.y < (rgb_image.rows / scale))
        {
            cv::circle(draw_img, cv::Point(uv.x, uv.y), 10, cv::Scalar(0, 0, 255), 3, 1);
            
            cv::rectangle(draw_img, cv::Point(uv_x1.x, uv_x2.y), cv::Point(uv_x2.x, uv_x1.y), cv::Scalar(0, 255, 0), 4);
            cv::circle(draw_img, cv::Point(uv_x1.x, uv_x2.y), 8, cv::Scalar(255, 255, 255), -1, 1);
            cv::circle(draw_img, cv::Point(uv_x2.x, uv_x1.y), 8, cv::Scalar(0, 0, 0), -1, 1);
            std::vector<cv::Point2d> uv_s;
            cv::Point2d new_x1(uv_x1.x, uv_x2.y), new_x2(uv_x2.x, uv_x1.y);
            uv_s.push_back(new_x1);
            uv_s.push_back(new_x2);
            // ROS_INFO_STREAM("new.x: " << new_x1.x << " x1.y: " << new_x1.y << "    x2.x: " << new_x2.x << " x2.y: " << new_x2.y);
            uv_points.push_back(uv_s);
        }
    }
    // std::cout << "uv_points size " << uv_points.size() << std::endl;
    
}

cv::Point2d Annotation_yolo::project3d_to_pixel(cv::Point3d xyz, sensor_msgs::CameraInfo cinfo_msg)
{
    cv::Point2d uv_rect;
    fx_ = cinfo_msg.K[0] * f_scale_;
    tx_ = cinfo_msg.K[1];
    cx_ = cinfo_msg.K[2] * cx_scale_;
    fy_ = cinfo_msg.K[4] * f_scale_;
    ty_ = cinfo_msg.K[3];
    cy_ = cinfo_msg.K[5] * cy_scale_;
    uv_rect.x = (fx_ * xyz.x + tx_) / xyz.z + cx_;
    uv_rect.y = (fy_ * -xyz.y + ty_) / xyz.z + cy_;
    return uv_rect;
}

void Annotation_yolo::paramter_set_bara(std::string base_tf_frame, int work_count)
{
    for (int i = 0; i < work_count; i++) {
        target_frames_.push_back(base_tf_frame + "_" + std::to_string(i));
    }
}

void Annotation_yolo::rotation_convert(geometry_msgs::TransformStamped source_tf, std::vector<geometry_msgs::TransformStamped> target_tfs, std::vector<cv::Point3d> &output_3d)
{
    geometry_msgs::Vector3 source_trans = source_tf.transform.translation;
    geometry_msgs::Quaternion source_quat = source_tf.transform.rotation;
    tf2::Quaternion q_rot;
    tf2::convert(source_quat, q_rot);
    for (int i = 0; i < target_tfs.size(); i++) {
        geometry_msgs::Vector3 target_trans = target_tfs[i].transform.translation;
        geometry_msgs::Quaternion target_quat = target_tfs[i].transform.rotation;
        tf2::Quaternion target_q_before(target_trans.x, target_trans.y, target_trans.z, 0);
        tf2::Quaternion target_rot;
        tf2::convert(target_quat, target_rot);
        tf2::Quaternion target_after = target_rot.inverse() * target_q_before * target_rot;
        // ROS_INFO_STREAM("translation:   x: " << target_after.x() << "   y: " << target_after.y() << " z: " << target_after.z());
        // double x = target_tfs[i].transform.translation.x - source_trans.x;
        // double y = target_tfs[i].transform.translation.y - source_trans.y;
        // double z = target_tfs[i].transform.translation.z - source_trans.z;
        double x = -target_after.x() - source_trans.x;
        double y = -target_after.y() - source_trans.y;
        double z = -target_after.z() - source_trans.z;
        tf2::Quaternion q_before(x, y, z, 0);
        // tf2::Quaternion q_after = q_rot * q_before * q_rot.inverse();
        tf2::Quaternion q_after = q_rot.inverse() * q_before * q_rot;
        tf2::Quaternion q_rotation;
        q_rotation.setRPY(0, 0, PI / 2);
        q_after = q_rotation * q_after * q_rotation.inverse();
        cv::Point3d out(q_after.x(), q_after.y(), q_after.z());
        output_3d.push_back(out);
    }
}

void Annotation_yolo::get_original_image(sensor_msgs::Image image1, cv::Mat &original_img)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge execption %s", e.what());
        // return;
    }
    original_img = cv_img_ptr->image;
}