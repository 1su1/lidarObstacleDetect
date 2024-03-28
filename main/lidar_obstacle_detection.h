#ifndef LIDAR_DETECTION_TRACK_H_
#define LIDAR_DETECTION_TRACK_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "bounding_box/bounding_box.h"
#include "euclidean_cluster/euclidean_cluster.h"
#include "ground_detector/patchworkpp/patchworkpp.hpp"
#include "pre_process/roi_clip/roi_clip.h"
#include "pre_process/voxel_grid_filter/voxel_grid_filter.h"
#include "visualization/visualize_detected_objects.h"

class LidarObstacleDetection
{

public:
    LidarObstacleDetection(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~LidarObstacleDetection(){};

private:
    void    ClusterCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
    void    PublishDetectedObjects(const autoware_msgs::CloudClusterArray& in_clusters,
                                   autoware_msgs::DetectedObjectArray&     detected_objects);
    int64_t GetTime();
    void    PublishCloud(const ros::Publisher* in_publisher, std_msgs::Header header,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);

private:
    RoiClip                     roi_clip_;
    VoxelGridFilter             voxel_grid_filter_;
    PatchWorkpp<pcl::PointXYZI> patch_work_;
    EuclideanCluster            cluster_;
    BoundingBox                 bounding_box_;
    VisualizeDetectedObjects    vdo_;

    ros::Publisher pub_clip_cloud_;
    ros::Publisher pub_ground_cloud_;
    ros::Publisher pub_noground_cloud_;
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_clusters_message_;
    ros::Publisher pub_detected_objects_;
    ros::Publisher pub_detected_3Dobjects_;
    ros::Publisher pub_cluster_visualize_markers_;
    ros::Publisher pub_3Dobjects_visualize_markers_;

    // 时间统计
    int64_t euclidean_time_;
    int64_t total_time_;
    int     counter_;
};

#endif
