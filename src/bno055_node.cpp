#include "bno055_driver.h"

#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle nh("~");
  ros::spin();

  return 0;
}
