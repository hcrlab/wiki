// A node that logs the average point of the given point cloud.
//
// Sample usage:
//   rosrun pcl_sample processor_node pointcloud2_in:=/camera/depth_registered/points
//
// Note: the PointCloud2 data from the lab Turtlebots uses a different
// coordinate system from the ROS convention. See the README on Github for more
// information.

// Technically, node_handle.h, and subscriber.h are transitively included with
// ros.h. However, it's generally a good C++ practice to always "include what
// you use" (IWYU).
#include "pcl_sample/processor.h" // pcl_sample::Processor
#include "ros/ros.h"              // ros::init, ros::spin, etc.
#include "ros/node_handle.h"      // ros::NodeHandle
#include "ros/subscriber.h"       // ros::Subscriber

using pcl_sample::Processor;

int main(int argc, char** argv) {
  // Initialize this program as a ROS node.
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  Processor processor(ros::Time::now());

  // Subscribe to /pointcloud2_in. Normally this would be remapped to another
  // topic, such as /camera/depth_registered/points. Calls the callback
  // function when a point cloud comes in.
  // Read more about the callback syntax at:
  // http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Callback_Types
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
    "pointcloud2_in", 1, &Processor::Callback, &processor);
  ros::spin();
}

