#include "roi_clip.h"

RoiClip::RoiClip(const RoiClipParams& params)
{
    roi_x_min_ = params.roi_x_min;
    roi_x_max_ = params.roi_x_max;
    roi_y_min_ = params.roi_y_min;
    roi_y_max_ = params.roi_y_max;
    roi_z_min_ = params.roi_z_min;
    roi_z_max_ = params.roi_z_max;

    vehicle_x_min_ = params.vehicle_x_min;
    vehicle_x_max_ = params.vehicle_x_max;
    vehicle_y_min_ = params.vehicle_y_min;
    vehicle_y_max_ = params.vehicle_y_max;
    vehicle_z_min_ = params.vehicle_z_min;
    vehicle_z_max_ = params.vehicle_z_max;
}

RoiClip::~RoiClip()
{
    // Do Nothing
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                                     pcl::PointCloud<pcl::PointXYZI>::Ptr&      out)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipCloud = ClipVehicle(in);
    if (clipCloud->points.size() > 0)
    {
        for (auto& p : clipCloud->points)
        {
            if (IsIn(p.x, roi_x_min_, roi_x_max_) && IsIn(p.y, roi_y_min_, roi_y_max_) &&
                IsIn(p.z, roi_z_min_, roi_z_max_))
            {
                out->push_back(p);
            }
            else
            {
            }
        }
        return out;
    }
    return nullptr;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::ClipVehicle(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in)
{
    if (in->points.size() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto& p : in->points)
        {
            if (IsIn(p.x, vehicle_x_min_, vehicle_x_max_) &&
                IsIn(p.y, vehicle_y_min_, vehicle_y_max_) &&
                IsIn(p.z, vehicle_z_min_, vehicle_z_max_))
            {
            }
            else
            {
                out->push_back(p);
            }
        }
        return out;
    }
    return nullptr;
}
