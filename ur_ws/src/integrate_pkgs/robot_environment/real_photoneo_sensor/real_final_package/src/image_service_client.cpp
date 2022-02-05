#include <ros/ros.h>
#include <denso_srvs/sensor_data_service.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <denso_srvs/sensor_data_serviceRequest.h>
#include <opencv4/opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fieek");
   
    ros::ServiceClient client;
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
     ros::Duration(1.0).sleep();
    std::string service_name;
    pnh.getParam("service_name", service_name);
    denso_srvs::sensor_data_service img;
    img.request.call = true;
    client = nh.serviceClient<denso_srvs::sensor_data_service>(service_name);
    cv::Mat img1;
    while (1) {
        client.call(img);
        cv_bridge::CvImagePtr input_bridge;
        input_bridge = cv_bridge::toCvCopy(img.response.image, sensor_msgs::image_encodings::BGR8);
        
        img1 = input_bridge->image;
        int sum = 0;
        int count = 0;
        for (int i = 0; i < img1.size().height; i+=10) {
            for (int j = 0; j < img1.size().width; j+=10) {
                sum =  (int)img1.at<cv::Vec3b>(i, j)[0] + sum;
                count++;
            }
        }
        double result = (double)(sum / count);
        if (result < 80)
            break;
    } 
    cv::imshow("mask_show", img1);
    cv::waitKey(1000);
    cv::imwrite("/home/ericlab/tsuchida/ros_package/integ_ws/src/tsuchida.jpg", img1);

    // ros::Publisher pub = nh.advertise<sensor_msgs::Image>("tsuchida", 10);
    // ros::Rate loop(10);
    // while (ros::ok())
    // {
    //     pub.publish(img.response.image);
    //     loop.sleep();
    // }
    return 0;
    
}