/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include "centroid-tracking.hpp"

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "zedroscenternode");

  // Create a ROS node handle
  ros::NodeHandle nh;

  zedroscenternode::Center center(nh);

  // Don't exit the program.
  ros::spin();

  return 0;
}
