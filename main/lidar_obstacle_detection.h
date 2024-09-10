#ifndef LIDAR_DETECTION_TRACK_H_
#define LIDAR_DETECTION_TRACK_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "autoware_auto_msgs/msg/bounding_box_array.hpp"
#include "autoware_auto_msgs/msg/detected_objects.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "bounding_box/bounding_box.h"
#include "euclidean_cluster/euclidean_cluster.h"
#include "ground_detector/patchworkpp/patchworkpp.hpp"
#include "pre_process/roi_clip/roi_clip.h"
#include "pre_process/voxel_grid_filter/voxel_grid_filter.h"
#include "visualization/visualize_detected_objects.h"

class LidarObstacleDetection : public rclcpp::Node
{
public:
    LidarObstacleDetection(const std::string node_name);
    ~LidarObstacleDetection() {};

private:
    void ClusterCallback(const sensor_msgs::msg::PointCloud2::SharedPtr in_sensor_cloud);
    void PublishDetectedObjects(const std::vector<autoware_auto_msgs::msg::BoundingBox>& in_clusters,
                                autoware_auto_msgs::msg::DetectedObjects& detected_objects);
    int64_t GetTime();
    void PublishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher, 
                      std_msgs::msg::Header header,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_to_publish_ptr);

private:
    RoiClip roi_clip_;
    VoxelGridFilter voxel_grid_filter_;
    PatchWorkpp<pcl::PointXYZI> patch_work_;
    EuclideanCluster cluster_;
    BoundingBox bounding_box_;
    VisualizeDetectedObjects vdo_;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clip_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_noground_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_cloud_;
    rclcpp::Publisher<autoware_auto_msgs::msg::BoundingBoxArray>::SharedPtr pub_clusters_message_;
    rclcpp::Publisher<autoware_auto_msgs::msg::DetectedObjects>::SharedPtr pub_detected_objects_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_detected_3Dobjects_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_visualize_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_3Dobjects_visualize_markers_;

    // 时间统计
    int64_t euclidean_time_;
    int64_t total_time_;
    int counter_;
};

#endif
