#include "lidar_obstacle_detection.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection_node");
    ros::NodeHandle        nh;
    ros::NodeHandle        pnh("~");
    LidarObstacleDetection core(nh, pnh);
    return 0;
}
