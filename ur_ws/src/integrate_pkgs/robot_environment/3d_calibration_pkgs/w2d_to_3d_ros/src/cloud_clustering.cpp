#include <w2d_to_3d_ros/cloud_clustering.hpp>


Clustering::Clustering(ros::NodeHandle& nh) : nh_(nh)
{
    ros::NodeHandle pnh("~");

    pub_.resize(CLUSTER_NUM);
    clusters_ros_.resize(CLUSTER_NUM);
    pnh.getParam("thresh_hold", thresh_hold);
    pnh.getParam("frame_id", frame_id);
    pnh.getParam("min_cluster_size", min_cluster_size);
    pnh.getParam("max_cluster_size", max_cluster_size);
    // ROS_INFO_STREAM("commen");

    for (std::vector<ros::Publisher>::iterator it = pub_.begin(); it != pub_.end(); ++it)
    {
        // ROS_INFO_STREAM("isiaia");
        std::stringstream ss;
        int index = it - pub_.begin();
        ss << "cloud_clustered" << index + 1;
        (*it) = nh_.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);
    }
}


void Clustering::operate()
{
    // ROS_INFO_STREAM("tsuchidasama");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::fromROSMsg(input_, *cloud_input_pcl_ptr);

    ROS_INFO_STREAM("operate");
    clusterKdTree(cloud_input_pcl_ptr, cluster_indices);
    extractCluster(cloud_input_pcl_ptr, cluster_indices);
}


void Clustering::publish()
{
    // ROS_INFO_STREAM("korea");
    for (std::vector<ros::Publisher>::iterator it=pub_.begin(); it != pub_.end(); ++it)
    {
        int index = it - pub_.begin();
        it->publish(clusters_ros_[index]);
    }
}


void Clustering::clusterKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr, std::vector<pcl::PointIndices> &cluster_indices)
{
    ROS_INFO_STREAM("kdtree");
    ROS_INFO_STREAM("thresh_hold" << thresh_hold);
    ROS_INFO_STREAM("cloud_filerd" << cloud_filtered_ptr->size());
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered_ptr);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(thresh_hold);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered_ptr);
    ec.extract(cluster_indices);
    ROS_INFO_STREAM("cluster_indices" << cluster_indices.size());
}


void Clustering::extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr, std::vector<pcl::PointIndices> &cluster_indices)
{
    ROS_INFO_STREAM("cluster");
    for (std::vector<pcl::PointIndices>::iterator it=cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        ROS_INFO_STREAM("cluster_for");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_pcl_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliars(new pcl::PointIndices(*it));

        extract.setInputCloud(cloud_filtered_ptr->makeShared());
        extract.setIndices(inliars);

        extract.setNegative(false);
        extract.filter(*cloud_cluster_pcl_ptr);

        ROS_INFO_STREAM("frame_id" << frame_id);

        sensor_msgs::PointCloud2 cloud_cluster_ros;
        pcl::toROSMsg(*cloud_cluster_pcl_ptr, cloud_cluster_ros);
        cloud_cluster_ros.header.frame_id = frame_id;

        int index = it - cluster_indices.begin();
        clusters_ros_[index] = cloud_cluster_ros;
    }
}
