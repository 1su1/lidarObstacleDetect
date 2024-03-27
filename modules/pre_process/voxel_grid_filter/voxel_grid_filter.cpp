#include "voxel_grid_filter.h"

VoxelGridFilter::VoxelGridFilter()
{
    // Do Nothing
}

VoxelGridFilter::VoxelGridFilter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
    /* Initialize tuning parameter */
    private_node_handle.param("/is_downsample", is_downsample_, true);
    private_node_handle.param("/leaf_size", leaf_size_, 0.1);
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
