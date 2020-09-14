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
      ROS_ERROR("Failed to initialize BNO055 Driver.");
    }
    if (bno055_driver_.setImuMode() < 0)
    {
      ROS_ERROR("Failed to set operation mode to IMU.");
    }
     
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  }
  
  void publishData()
  {
    if (bno055_driver_.getAcc() < 0)
    {
      ROS_ERROR("Failed to get accelerometer data.");
    }
    if (bno055_driver_.getMag() < 0)
    {
      ROS_ERROR("Failed to get magnometer data.");
    }
    if (bno055_driver_.getGyr() < 0)
    {
      ROS_ERROR("Failed to get gyroscope data.");
    }
    if (bno055_driver_.getEul() < 0)
    {
      ROS_ERROR("Failed to get euler angles data.");
    }
    if (bno055_driver_.getQua() < 0)
    {
      ROS_ERROR("Failed to get quaternions data.");
    }
    
    ros::Time time_stamp = ros::Time::now();

    imu_msg_.header.stamp = time_stamp;
    
    imu_msg_.orientation.x = bno055_driver_.data_.qua_x_; 
    imu_msg_.orientation.y = bno055_driver_.data_.qua_y_;
    imu_msg_.orientation.z = bno055_driver_.data_.qua_z_;
    imu_msg_.orientation.w = bno055_driver_.data_.qua_w_;

    imu_msg_.angular_velocity.x = bno055_driver_.data_.gyr_x_;
    imu_msg_.angular_velocity.y = bno055_driver_.data_.gyr_y_;
    imu_msg_.angular_velocity.z = bno055_driver_.data_.gyr_z_;

    imu_msg_.linear_acceleration.x = bno055_driver_.data_.acc_x_;
    imu_msg_.linear_acceleration.y = bno055_driver_.data_.acc_y_;
    imu_msg_.linear_acceleration.z = bno055_driver_.data_.acc_z_;
    
    mag_msg_.header.stamp = time_stamp;
    
    mag_msg_.magnetic_field.x = bno055_driver_.data_.mag_x_;
    mag_msg_.magnetic_field.y = bno055_driver_.data_.mag_y_;
    mag_msg_.magnetic_field.z = bno055_driver_.data_.mag_z_;

    imu_pub_.publish(imu_msg_);
    mag_pub_.publish(mag_msg_);
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
