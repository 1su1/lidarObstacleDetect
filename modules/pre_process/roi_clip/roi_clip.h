#ifndef ROI_CLIP_H
#define ROI_CLIP_H

#include <boost/variant.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>


class RoiClip
{
public:
    RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
    ~RoiClip();
    pcl::PointCloud<pcl::PointXYZI>::Ptr    GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                                   pcl::PointCloud<pcl::PointXYZI>::Ptr&      out);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GetFusionROI(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const sensor_msgs::ImageConstPtr& image_msg,
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& out);

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr ClipVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr in);

    bool                            PointOutVechicle(const pcl::PointXYZI& input_point);
    bool                            PointInROI(const pcl::PointXYZI& input_point);
    boost::variant<cv::Point, bool> GetPixelInCamera(const pcl::PointXYZI&             input_point,
                                                     const sensor_msgs::ImageConstPtr& image_msg);
    void                            DrawPointWithCamera(pcl::PointXYZI&                          input_point,
                                                        const sensor_msgs::ImageConstPtr&        image_msg,
                                                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& out);

private:
    double             roi_x_min_;
    double             roi_x_max_;
    double             roi_y_min_;
    double             roi_y_max_;
    double             roi_z_min_;
    double             roi_z_max_;
    bool               is_reverse_;
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