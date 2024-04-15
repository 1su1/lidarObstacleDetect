#include "lidar_obstacle_detection.h"

LidarObstacleDetection::LidarObstacleDetection(ros::NodeHandle nh, ros::NodeHandle pnh)
    : roi_clip_(nh, pnh)
    , voxel_grid_filter_(nh, pnh)
    , cluster_(nh, pnh)
    , bounding_box_(pnh)
    , patch_work_(&pnh)
{
    ros::Subscriber sub =
        nh.subscribe("/os1_cloud_node/points", 1, &LidarObstacleDetection::ClusterCallback, this);

    pub_clip_cloud_       = nh.advertise<sensor_msgs::PointCloud2>("/points_clip", 1);
    pub_noground_cloud_   = nh.advertise<sensor_msgs::PointCloud2>("/noground_points", 1);
    pub_ground_cloud_     = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);
    pub_cluster_cloud_    = nh.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
    pub_clusters_message_ = nh.advertise<autoware_msgs::CloudClusterArray>(
        "/detection/lidar_detector/cloud_clusters", 1);
    pub_detected_objects_ =
        nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
    pub_cluster_visualize_markers_ =
        nh.advertise<visualization_msgs::MarkerArray>("/visualize/cluster_markers", 1);
    ros::spin();
}

void LidarObstacleDetection::ClusterCallback(
    const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
    std_msgs::Header                     header = in_sensor_cloud->header;
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clip_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*in_sensor_cloud, *in_cloud_ptr);

    // 提取ROI
    int64_t tm0 = GetTime();
    roi_clip_.GetROI(in_cloud_ptr, clip_cloud_ptr);
    std::cout << "clip_cloud_ptr " << clip_cloud_ptr->points.size() << std::endl;
    int64_t tm1 = GetTime();
    ROS_INFO("ROI_Clip cost time:%ld ms", (tm1 - tm0) / 1000);

    // 下采样
    voxel_grid_filter_.Downsample(clip_cloud_ptr, downsampled_cloud_ptr);

    // 地面分割
    double time_taken;
    patch_work_.estimate_ground(
        *downsampled_cloud_ptr, *ground_cloud_ptr, *noground_cloud_ptr, time_taken);
    int64_t tm2 = GetTime();
    ROS_INFO("remove ground cost time:%ld ms", (tm2 - tm1) / 1000);

    // 聚类
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointsVector;
    cluster_.segmentByDistance(noground_cloud_ptr, outCloudPtr, pointsVector);
    int64_t tm3 = GetTime();
    ROS_INFO("euclidean cluster cost time:%ld ms", (tm3 - tm2) / 1000);
    ROS_INFO("total cost time:%ld ms", (tm3 - tm0) / 1000);

    // 获取bounding_box信息
    autoware_msgs::CloudClusterArray inOutClusters;
    bounding_box_.getBoundingBox(header, pointsVector, inOutClusters);
    autoware_msgs::DetectedObjectArray detected_objects;
    PublishDetectedObjects(inOutClusters, detected_objects);

    // 可视化
    visualization_msgs::MarkerArray visualize_markers;
    vdo_.visualizeDetectedObjs(detected_objects, visualize_markers);
    pub_cluster_visualize_markers_.publish(visualize_markers);

    // 发布topic
    PublishCloud(&pub_clip_cloud_, in_sensor_cloud->header, clip_cloud_ptr);
    PublishCloud(&pub_noground_cloud_, in_sensor_cloud->header, noground_cloud_ptr);
    PublishCloud(&pub_cluster_cloud_, in_sensor_cloud->header, outCloudPtr);
    PublishCloud(&pub_ground_cloud_, in_sensor_cloud->header, ground_cloud_ptr);

    euclidean_time_ += tm3 - tm2;
    total_time_ += tm3 - tm0;
    counter_++;
    if (counter_ % 100 == 0)
    {
        ROS_INFO("[INFO] euclidean cluster average time per hundred times:%ld ms",
                 euclidean_time_ / 100000);
        ROS_INFO("[INFO] total average time per hundred times:%ld ms", total_time_ / 100000);
        euclidean_time_ = 0.;
        total_time_     = 0.;
    }
}

void LidarObstacleDetection::PublishDetectedObjects(
    const autoware_msgs::CloudClusterArray& in_clusters,
    autoware_msgs::DetectedObjectArray&     detected_objects)
{
    detected_objects.header = in_clusters.header;
    for (size_t i = 0; i < in_clusters.clusters.size(); i++)
    {
        autoware_msgs::DetectedObject detected_object;
        detected_object.header      = in_clusters.header;
        detected_object.label       = "unknown";
        detected_object.score       = 1.;
        detected_object.space_frame = in_clusters.header.frame_id;
        detected_object.pose        = in_clusters.clusters[i].bounding_box.pose;

        detected_object.dimensions  = in_clusters.clusters[i].dimensions;
        detected_object.pointcloud  = in_clusters.clusters[i].cloud;
        detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
        detected_object.valid       = true;

        detected_objects.objects.push_back(detected_object);
    }

    pub_detected_objects_.publish(detected_objects);
}

int64_t LidarObstacleDetection::GetTime()
{
    struct timeval tm;
    gettimeofday(&tm, 0);
    int64_t re = (((int64_t)tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
    return re;
}

void LidarObstacleDetection::PublishCloud(
    const ros::Publisher* in_publisher, std_msgs::Header header,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = header;
    in_publisher->publish(cloud_msg);
}
