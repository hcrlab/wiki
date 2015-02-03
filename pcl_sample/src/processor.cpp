// Implementation of the processor library.
// See processor.h for documentation.

// See processor_node.cpp for explanations of what each header provides.
#include "pcl_sample/processor.h"

#include "pcl/point_types.h"                 // pcl::PointXYZRGB
#include "pcl_conversions/pcl_conversions.h" // pcl::fromROSMsg
#include "pcl_ros/point_cloud.h"             // pcl::PointCloud
#include "ros/console.h"                     // ROS_INFO
#include "sensor_msgs/PointCloud2.h"         // sensor_msgs::PointCloud2

#include <math.h> // is_nan

namespace pcl_sample {
Processor::Processor(const ros::Time& prev_time)
  : prev_time_(prev_time),
    time_taken_(ros::Duration(0)),
    num_calls_(0) {
}

void Processor::Average(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    pcl::PointXYZRGB* average) { 
  double average_x = 0;
  double average_y = 0;
  double average_z = 0;
  for (const pcl::PointXYZRGB& point : cloud.points) {
    // Some points may have NaN values for position.
    if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
      continue;
    }
    average_x += point.x;
    average_y += point.y;
    average_z += point.z;
  }
  average->x = average_x / cloud.points.size();
  average->y = average_y / cloud.points.size();
  average->z = average_z / cloud.points.size();
  
  ros::Time current_time = ros::Time::now();
  time_taken_ += current_time - prev_time_;
  prev_time_ = current_time;
  num_calls_ += 1;
}

void Processor::Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // Use PCL's conversion.
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointXYZRGB average;

  Average(cloud, &average);

  // Log the average point at the INFO level. You can specify other logging
  // levels using ROS_DEBUG, ROS_WARN, etc. Search for "ros logging" online.
  ROS_INFO("x: %f y: %f z: %f, time/point: %fs",
    average.x, average.y, average.z, time_taken_.toSec() / num_calls_);
}

}
