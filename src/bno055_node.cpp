#include "bno055_driver/bno055_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace bno055
{
class Bno055Node
{
public:
  Bno055Node(const ros::NodeHandle& nh) : nh_(nh)
  {
    if (bno055_driver_.initI2c() < 0)
    {
      printf("Failed to initialize BNO055 Driver.\n");
    }
    if (bno055_driver_.setImuMode() < 0)
    {
      printf("Failed to set operation mode to IMU.\n");
    }
  }
  
  void publishData()
  {
    if (bno055_driver_.getAcc() < 0)
    {
      printf("Failed to get accelerometer data.\n");
    }
    else 
    {
      printf("ACC X: %f, ACC Y: %f, ACC Z: %f\n", bno055_driver_.acc_data_.acc_x_, bno055_driver_.acc_data_.acc_y_, bno055_driver_.acc_data_.acc_z_);
    }
  }

  ~Bno055Node()
  {}
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  bno055::Bno055Driver bno055_driver_;
};
} // namespace bno055

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle nh("~");
  bno055::Bno055Node bno055_node(nh);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    bno055_node.publishData();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
