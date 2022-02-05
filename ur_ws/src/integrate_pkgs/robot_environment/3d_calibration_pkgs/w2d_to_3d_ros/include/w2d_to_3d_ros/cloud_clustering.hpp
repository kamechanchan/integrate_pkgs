#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <w2d_to_3d_ros/abstract_class.hpp>


class Clustering : public cloud_abstract
{
private:
    ros::NodeHandle nh_;
    std::vector<ros::Publisher> pub_;
    std::vector<sensor_msgs::PointCloud2> clusters_ros_;
    std::string output_topic_;
    std::string frame_id;
    double thresh_hold;
    int min_cluster_size;
    int max_cluster_size;
    static const int CLUSTER_NUM = 3;
    void clusterKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>&);
    void extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>&);

public:
    Clustering(ros::NodeHandle&);
    void operate();
    void publish();
};