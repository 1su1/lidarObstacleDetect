#ifndef ROI_CLIP_H
#define ROI_CLIP_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

class RoiClip
{
public:
    RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    ~RoiClip();
    pcl::PointCloud<pcl::PointXYZI>::Ptr GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                                pcl::PointCloud<pcl::PointXYZI>::Ptr&      out);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ClipVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in);

    bool point_in_camera(const pcl::PointXYZRGBA& point);
    void draw_point_with_camera();

private:
    double             roi_x_min_;
    double             roi_x_max_;
    double             roi_y_min_;
    double             roi_y_max_;
    double             roi_z_min_;
    double             roi_z_max_;
    double             vehicle_x_min_;
    double             vehicle_x_max_;
    double             vehicle_y_min_;
    double             vehicle_y_max_;
    double             vehicle_z_min_;
    double             vehicle_z_max_;
    Eigen::Quaternionf lidar_to_camera_rotation_{-0.50507811, 0.51206185, 0.49024953, -0.49228464};
    Eigen::Vector3f    lidar_to_camera_translation_{-0.13165462, 0.03870398, -0.17253834};
    cv::Mat            camera_matrix_;

    bool IsIn(const float x, const float x_min, const float x_max)
    {
        return (x < x_max) && (x > x_min);
    }
};

#endif