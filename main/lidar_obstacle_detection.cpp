#include "lidar_obstacle_detection.h"

LidarObstacleDetection::LidarObstacleDetection(const std::string& node_name)
    : Node(node_name)
    , roi_clip_(shared_from_this())
    , voxel_grid_filter_(shared_from_this())
    , patch_work_(shared_from_this())
{
    // 创建订阅者
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/os1_cloud_node/points",
        1,
        std::bind(&LidarObstacleDetection::ClusterCallback, this, std::placeholders::_1));

    // 创建发布者
    pub_clip_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_clip", 1);
    pub_noground_cloud_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/noground_points", 1);
    pub_ground_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_points", 1);
    pub_cluster_cloud_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_cluster", 1);
    pub_clusters_message_ = this->create_publisher<autoware_auto_msgs::msg::BoundingBoxArray>(
        "/detection/lidar_detector/cloud_clusters", 1);
    pub_detected_objects_ = this->create_publisher<autoware_auto_msgs::msg::DetectedObjects>(
        "/detection/lidar_detector/objects", 1);
    pub_cluster_visualize_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualize/cluster_markers", 1);
}

void LidarObstacleDetection::ClusterCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr in_sensor_cloud)
{
    std_msgs::msg::Header                header = in_sensor_cloud->header;
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
    RCLCPP_INFO(this->get_logger(), "clip_cloud_ptr %zu points", clip_cloud_ptr->points.size());
    int64_t tm1 = GetTime();
    RCLCPP_INFO(this->get_logger(), "ROI_Clip cost time: %ld ms", (tm1 - tm0) / 1000);

    // 下采样
    voxel_grid_filter_.Downsample(clip_cloud_ptr, downsampled_cloud_ptr);

    // 地面分割
    double time_taken;
    patch_work_.estimate_ground(
        *downsampled_cloud_ptr, *ground_cloud_ptr, *noground_cloud_ptr, time_taken);
    int64_t tm2 = GetTime();
    RCLCPP_INFO(this->get_logger(), "Remove ground cost time: %ld ms", (tm2 - tm1) / 1000);

    // 聚类
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointsVector;
    cluster_.segmentByDistance(noground_cloud_ptr, outCloudPtr, pointsVector);
    int64_t tm3 = GetTime();
    RCLCPP_INFO(this->get_logger(), "Euclidean cluster cost time: %ld ms", (tm3 - tm2) / 1000);
    RCLCPP_INFO(this->get_logger(), "Total cost time: %ld ms", (tm3 - tm0) / 1000);

    // 获取bounding_box信息
    autoware_auto_msgs::msg::BoundingBoxArray inOutClusters;
    bounding_box_.getBoundingBox(header, pointsVector, inOutClusters);
    autoware_auto_msgs::msg::DetectedObjects detected_objects;
    PublishDetectedObjects(inOutClusters, detected_objects);

    // 可视化
    visualization_msgs::msg::MarkerArray visualize_markers;
    vdo_.visualizeDetectedObjs(detected_objects, visualize_markers);
    pub_cluster_visualize_markers_->publish(visualize_markers);

    // 发布topic
    PublishCloud(pub_clip_cloud_, header, clip_cloud_ptr);
    PublishCloud(pub_noground_cloud_, header, noground_cloud_ptr);
    PublishCloud(pub_cluster_cloud_, header, outCloudPtr);
    PublishCloud(pub_ground_cloud_, header, ground_cloud_ptr);

    euclidean_time_ += tm3 - tm2;
    total_time_ += tm3 - tm0;
    counter_++;
    if (counter_ % 100 == 0)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Euclidean cluster average time per hundred: %ld ms",
                    euclidean_time_ / 100000);
        RCLCPP_INFO(
            this->get_logger(), "Total average time per hundred: %ld ms", total_time_ / 100000);
        euclidean_time_ = 0;
        total_time_     = 0;
    }
}

void LidarObstacleDetection::PublishDetectedObjects(
    const autoware_auto_msgs::msg::BoundingBoxArray& in_clusters,
    autoware_auto_msgs::msg::DetectedObjects&        detected_objects)
{
    detected_objects.header = in_clusters.header;
    for (const auto& cluster : in_clusters.boxes)
    {
        autoware_auto_msgs::msg::DetectedObject detected_object;
        detected_object.header      = in_clusters.header;
        detected_object.label       = "unknown";
        detected_object.score       = 1.0;
        detected_object.space_frame = in_clusters.header.frame_id;
        detected_object.pose        = cluster.centroid;
        detected_object.dimensions  = cluster.size;
        detected_object.valid       = true;

        detected_objects.objects.push_back(detected_object);
    }

    pub_detected_objects_->publish(detected_objects);
}

int64_t LidarObstacleDetection::GetTime()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void LidarObstacleDetection::PublishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
    std_msgs::msg::Header header, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_to_publish_ptr)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = header;
    publisher->publish(cloud_msg);
}
