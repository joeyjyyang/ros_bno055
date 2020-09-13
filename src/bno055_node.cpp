#include "bno055_driver/bno055_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

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
     
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  }
  
  void publishData()
  {
    if (bno055_driver_.getAcc() < 0)
    {
      printf("Failed to get accelerometer data.\n");
    }
    else 
    {
      printf("ACC X: %f, ACC Y: %f, ACC Z: %f\n", bno055_driver_.data_.acc_x_, bno055_driver_.data_.acc_y_, bno055_driver_.data_.acc_z_);
    }
    
    if (bno055_driver_.getMag() < 0)
    {
      printf("Failed to get magnometer data.\n");
    }
    else 
    {
      printf("MAG X: %f, MAG Y: %f, MAG Z: %f\n", bno055_driver_.data_.mag_x_, bno055_driver_.data_.mag_y_, bno055_driver_.data_.mag_z_);
    }
    
    if (bno055_driver_.getGyr() < 0)
    {
      printf("Failed to get gyroscope data.\n");
    }
    else 
    {
      printf("GYR X: %f, GYR Y: %f, GYR Z: %f\n", bno055_driver_.data_.gyr_x_, bno055_driver_.data_.gyr_y_, bno055_driver_.data_.gyr_z_);
    }
    
    if (bno055_driver_.getEul() < 0)
    {
      printf("Failed to get euler angles data.\n");
    }
    else 
    {
      printf("EUL HEADING: %f, EUL ROLL: %f, EUL PITCH: %f\n", bno055_driver_.data_.eul_heading_, bno055_driver_.data_.eul_roll_, bno055_driver_.data_.eul_pitch_);
    }

    if (bno055_driver_.getQua() < 0)
    {
      printf("Failed to get quaternions data.\n");
    }
    else 
    {
      printf("QUA W: %f, QUA X: %f, QUA Y: %f, QUA Z: %f.\n", bno055_driver_.data_.qua_w_, bno055_driver_.data_.qua_x_, bno055_driver_.data_.qua_y_, bno055_driver_.data_.qua_z_);
    }

    printf("---------------------------------------\n"); 
  }

  ~Bno055Node()
  {}
private:
  /* ROS */
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::MagneticField mag_msg_;
  /* BNO055 */
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
