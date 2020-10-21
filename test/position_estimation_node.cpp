#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void chatterCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
 ROS_INFO("I heard: [%d]", imu_msg->header.seq);
 ROS_INFO("Z: [%f]", imu_msg->linear_acceleration.x);
 ROS_INFO("Y: [%f]", imu_msg->linear_acceleration.y); 
 ROS_INFO("Z: [%f]", imu_msg->linear_acceleration.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_estimation_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("bno055_node/imu", 1, chatterCallback);
  
  ros::spin();

  return 0;
}

