#include "voxel_grid_filter.h"

VoxelGridFilter::VoxelGridFilter(const DownsampleParams& params)
{
    is_downsample_ = params.is_downsample;
    leaf_size_     = params.leaf_size;
}

VoxelGridFilter::~VoxelGridFilter()
{
    // Do Nothing
}

void VoxelGridFilter::Downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr&       out_cloud)
{
    // // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
    if (leaf_size_ >= 0.1)
    {
        // Downsampling the velodyne scan using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(in_cloud);
        // 参数为float
        voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        voxel.filter(*out_cloud);
    }
    else
    {
        out_cloud = in_cloud;
    }
}
