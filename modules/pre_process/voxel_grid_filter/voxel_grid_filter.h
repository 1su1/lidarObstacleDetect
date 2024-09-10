#ifndef DOWNSAMPLER_H
#define DOWNSAMPLER_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <unistd.h>

struct DownsampleParams
{
    bool  is_downsample;
    float leaf_size;
};

class VoxelGridFilter
{
public:
    VoxelGridFilter(const DownsampleParams& params);
    ~VoxelGridFilter();

    void Downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr&       out_cloud);

private:
    // 下采样：体素滤波参数
    double leaf_size_;
    bool   is_downsample_;
};

#endif   // DOWNSAMPLER_H
