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
  
  ~Bno055Node()
  {}
private:
  ros::NodeHandle nh_;
  bno055::Bno055Driver bno055_driver_;
};
} // namespace bno055

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle nh("~");
  bno055::Bno055Node bno055_node(nh);
  ros::spin();

  return 0;
}
