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

    private_node_handle.param("/is_reverse", is_reverse_, true);

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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RoiClip::GetFusionROI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const sensor_msgs::ImageConstPtr& image_msg,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& out)
{
    out->resize(in->size());
    if (in->points.size() > 0)
    {
        for (auto& p : in->points)
        {
            if (PointOutVechicle(p) && PointInROI(p))
            {
                DrawPointWithCamera(p, image_msg, out);
            }
            else
            {
            }
        }
        return out;
    }
    else
    {
        ROS_ERROR("GetFusionROI fail");
        return nullptr;
    }
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

bool RoiClip::PointOutVechicle(const pcl::PointXYZI& input_point)
{
    if (IsIn(input_point.x, vehicle_x_min_, vehicle_x_max_) &&
        IsIn(input_point.y, vehicle_y_min_, vehicle_y_max_) &&
        IsIn(input_point.z, vehicle_z_min_, vehicle_z_max_))
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool RoiClip::PointInROI(const pcl::PointXYZI& input_point)
{
    if (IsIn(input_point.x, roi_x_min_, roi_x_max_) &&
        IsIn(input_point.y, roi_y_min_, roi_y_max_) && IsIn(input_point.z, roi_z_min_, roi_z_max_))
    {
        return true;
    }
    else
    {
        return false;
    }
}

boost::variant<cv::Point, bool> RoiClip::GetPixelInCamera(
    const pcl::PointXYZI& input_point, const sensor_msgs::ImageConstPtr& image_msg)
{
    if (input_point.x > 0 && is_reverse_)
    {
        return false;
    }
    // 将输入点转换为 Eigen::Vector3f 类型，以便进行减法运算
    Eigen::Vector3f input_point_vec(input_point.x, input_point.y, input_point.z);

    // 根据标定结果将点从激光雷达坐标系转换到摄像头坐标系
    Eigen::Vector3f camera_point =
        lidar_to_camera_rotation_.inverse() * (input_point_vec - lidar_to_camera_translation_);

    // 计算像素坐标
    cv::Mat camera_point_mat(3, 1, CV_64F);
    camera_point_mat.at<double>(0, 0) = camera_point[0] / camera_point[2];
    camera_point_mat.at<double>(1, 0) = camera_point[1] / camera_point[2];
    camera_point_mat.at<double>(2, 0) = 1.0;

    cv::Mat   pixel = camera_matrix_ * camera_point_mat;
    cv::Point pixel_point(pixel.at<double>(0, 0), pixel.at<double>(1, 0));

    // 检查像素坐标是否在摄像头视野中,
    // 判断条件应该使用ROS消息中图像分辨率，而不是用通过焦距计算出来的图像尺寸，否则会发生未知错误
    if (pixel_point.x >= 0 && pixel_point.y >= 0 && pixel_point.x < image_msg->width &&
        pixel_point.y < image_msg->height)
    {
        return pixel_point;
    }
    else
    {
        return false;
    }
}

void RoiClip::DrawPointWithCamera(pcl::PointXYZI&                          input_point,
                                  const sensor_msgs::ImageConstPtr&        image_msg,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& out)
{
    // 获取激光雷达点的坐标
    Eigen::Vector3f point_lidar(input_point.x, input_point.y, input_point.z);

    // 获取图像消息中的颜色信息
    const int      image_width  = image_msg->width;
    const int      image_height = image_msg->height;
    const int      channels     = image_msg->step / image_msg->width;   // 通道数
    const uint8_t* image_data   = &image_msg->data[0];

    auto result = GetPixelInCamera(input_point, image_msg);
    if (result.type() == typeid(cv::Point))
    {
        pcl::PointXYZRGBA output_point;
        output_point.x = input_point.x;
        output_point.y = input_point.y;
        output_point.z = input_point.z;
        output_point.a = input_point.intensity;

        cv::Point pixel_point = boost::get<cv::Point>(result);
        // 计算图像中像素的索引
        int index = (pixel_point.y * image_width + pixel_point.x) * channels;
        // 获取对应像素的颜色值,将颜色信息应用到激光雷达点云中对应的点上
        output_point.r = image_data[index + 2];   // R 通道
        output_point.g = image_data[index + 1];   // G 通道
        output_point.b = image_data[index];       // B 通道
        out->emplace_back(output_point);
    }
    else if (result.type() == typeid(bool))
    {
        // 未在摄像头视野范围内的点着色为白色
        pcl::PointXYZRGBA output_point;
        output_point.x = input_point.x;
        output_point.y = input_point.y;
        output_point.z = input_point.z;
        output_point.r = 255;   // R 通道
        output_point.g = 0;     // G 通道
        output_point.b = 0;     // B 通道
        output_point.a = input_point.intensity;
        out->emplace_back(output_point);
    }
}