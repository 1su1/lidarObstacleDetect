#include "roi_clip.h"

RoiClip::RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
    private_node_handle.param("/roi_x_min", roi_x_min_, -80.0);
    private_node_handle.param("/roi_x_max", roi_x_max_, 20.0);
    private_node_handle.param("/roi_y_min", roi_y_min_, -20.0);
    private_node_handle.param("/roi_y_max", roi_y_max_, 20.0);
    private_node_handle.param("/roi_z_min", roi_z_min_, -2.0);
    private_node_handle.param("/roi_z_max", roi_z_max_, 2.0);
    // 转换到车辆坐标系下后，将车身点云切除，车辆坐标系中心为后轴中心０点
    // 坐标系参数不确定，暂时不转换
    private_node_handle.param("/vehicle_x_min", vehicle_x_min_, -0.8);
    private_node_handle.param("/vehicle_x_max", vehicle_x_max_, 0.8);
    private_node_handle.param("/vehicle_y_min", vehicle_y_min_, -0.8);
    private_node_handle.param("/vehicle_y_max", vehicle_y_max_, 0.8);
    private_node_handle.param("/vehicle_z_min", vehicle_z_min_, -0.6);
    private_node_handle.param("/vehicle_z_max", vehicle_z_max_, 0.4);

    double fx = 2813.643275;
    double fy = 2808.326079;
    double cx = 969.285772;
    double cy = 624.049972;

    // 构建相机内参矩阵
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    ROS_INFO("lidar_to_camera_rotation:: %f, %f, %f, %f",
             lidar_to_camera_rotation_.w(),
             lidar_to_camera_rotation_.x(),
             lidar_to_camera_rotation_.y(),
             lidar_to_camera_rotation_.z());
    ROS_INFO("lidar_to_camera_translation:: %f, %f, %f",
             lidar_to_camera_translation_.x(),
             lidar_to_camera_translation_.y(),
             lidar_to_camera_translation_.z());
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
        ROS_INFO("GetROI sucess");
        return out;
    }

    ROS_ERROR("GetROI fail");
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

bool RoiClip::point_in_camera(const pcl::PointXYZRGBA& point)
{
    // // 根据标定结果将点从激光雷达坐标系转换到摄像头坐标系
    // Eigen::Vector3f pointCamera =
    //     lidar_to_camera_rotation_.inverse() * (point - lidar_to_camera_translation_);

    // // 计算像素坐标
    // cv::Mat point(3, 1, CV_64F);
    // point.at<double>(0, 0) = pointCamera[0] / pointCamera[2];
    // point.at<double>(1, 0) = pointCamera[1] / pointCamera[2];
    // point.at<double>(2, 0) = 1.0;

    // cv::Mat   pixel = camera_matrix_ * point;
    // cv::Point p(pixel.at<double>(0, 0), pixel.at<double>(1, 0));

    // // 检查点是否在摄像头视野中
    // if (p.x >= 0 && p.y >= 0 && p.x < camera_matrix_.at<double>(0, 2) * 2 &&
    //     p.y < camera_matrix_.at<double>(1, 2) * 2)
    // {
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
}

void RoiClip::draw_point_with_camera()
{
    // for (size_t i = 0; i < cloud->points.size(); ++i)
    // {
    //     // 获取激光雷达点的坐标
    //     Eigen::Vector3f pointLidar(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    //     // 将点投影到摄像头图像中
    //     cv::Point pixelPoint;
    //     if (projectPointToImage(pointLidar, q, t, cameraMatrix, pixelPoint))
    //     {
    //         // 获取最近的像素点的颜色信息
    //         cv::Vec3b color = image.at<cv::Vec3b>(pixelPoint);

    //         // 将颜色信息应用到激光雷达点云中对应的点上
    //         cloud->points[i].r = color[2];
    //         cloud->points[i].g = color[1];
    //         cloud->points[i].b = color[0];
    //     }
    // }
}