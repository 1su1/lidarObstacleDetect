#ifndef DOWNSAMPLER_H
#define DOWNSAMPLER_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <unistd.h>

#include <Eigen/Eigen>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>

class VoxelGridFilter
{
public:
    VoxelGridFilter();
    VoxelGridFilter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    ~VoxelGridFilter();

    void Downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr&       out_cloud);

private:
    // 下采样：体素滤波参数
    double leaf_size_;
    bool   is_downsample_;
};

#endif   // DOWNSAMPLER_H
