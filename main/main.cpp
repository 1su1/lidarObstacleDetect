#include "lidar_obstacle_detection.h"

int main(int argc, char** argv)
{
    // 初始化ROS2系统
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<LidarObstacleDetection>("lidar_obstacle_detection");

    // 执行节点功能（spin节点以处理回调）
    rclcpp::spin(node);

    // 关闭ROS2系统
    rclcpp::shutdown();
    return 0;
}
