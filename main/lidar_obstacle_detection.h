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
#include "ground_detector/patchwork/patchwork.h"
#include "pre_process/roi_clip/roi_clip.h"
#include "pre_process/voxel_grid_filter/voxel_grid_filter.h"
#include "visualization/visualize_detected_objects.h"

class lidarObstacleDetection
{
public:
    lidarObstacleDetection(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~lidarObstacleDetection(){};

    RoiClip          roi_clip_;
    VoxelGridFilter  voxel_grid_filter_;
    PatchWork        patch_work_;
    EuclideanCluster cluster_;

    BoundingBox              bounding_box_;
    VisualizeDetectedObjects vdo_;

    ros::Publisher _pub_clip_cloud;
    ros::Publisher _pub_ground_cloud;
    ros::Publisher _pub_noground_cloud;
    ros::Publisher _pub_cluster_cloud;
    ros::Publisher _pub_clusters_message;

    ros::Publisher _pub_detected_objects;
    ros::Publisher _pub_detected_3Dobjects;

    ros::Publisher _pub_cluster_visualize_markers;
    ros::Publisher _pub_3Dobjects_visualize_markers;

    VisualizeDetectedObjects vdto;

private:
    void ClusterCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
    void publishDetectedObjects(const autoware_msgs::CloudClusterArray& in_clusters,
                                autoware_msgs::DetectedObjectArray&     detected_objects);
};

#endif
