#ifndef _VISUALIZEDETECTEDOBJECTS_H
#define _VISUALIZEDETECTEDOBJECTS_H

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <tf2/transform_datatypes.h>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>

#define __APP_NAME__ "visualize_detected_objects"

class VisualizeDetectedObjects
{
private:
    const double arrow_height_;
    const double label_height_;
    const double object_max_linear_size_ = 50.;
    double       object_speed_threshold_;
    double       arrow_speed_threshold_;
    double       marker_display_duration_;

    int marker_id_;

    std_msgs::ColorRGBA label_color_, cube_color_, box_color_, hull_color_, arrow_color_,
        centroid_color_, model_color_;

    std::string input_topic_, ros_namespace_;

    // ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_detected_objects_;

    ros::Publisher publisher_markers_;

    visualization_msgs::MarkerArray ObjectsToLabels(
        const autoware_msgs::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray ObjectsToArrows(
        const autoware_msgs::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray ObjectsToCubes(
        const autoware_msgs::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray ObjectsToBoxes(
        const autoware_msgs::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray ObjectsToModels(
        const autoware_msgs::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray ObjectsToHulls(
        const autoware_msgs::DetectedObjectArray& in_objects);

    visualization_msgs::MarkerArray ObjectsToCentroids(
        const autoware_msgs::DetectedObjectArray& in_objects);

    std::string ColorToString(const std_msgs::ColorRGBA& in_color);

    void DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray& in_objects);

    bool IsObjectValid(const autoware_msgs::DetectedObject& in_object);

    float CheckColor(double value);

    float CheckAlpha(double value);

    std_msgs::ColorRGBA ParseColor(const std::vector<double>& in_color);

public:
    VisualizeDetectedObjects();

    void visualizeDetectedObjs(const autoware_msgs::DetectedObjectArray& in_objects,
                               visualization_msgs::MarkerArray&          visualization_markers);
};

#endif   // _VISUALIZEDETECTEDOBJECTS_H
