/*
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    ROS node that wraps around the ROS-agnostic Linux hardware driver for the BNO055 sensor,
    which instantiates the driver object and publishes sensor data to ROS.
*/

#include "ros_bno055/bno055_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <ros_bno055/OrientationEuler.h>
#include <ros_bno055/Gravity.h>

namespace bno055
{
class Bno055Node
{
public:
  Bno055Node(const ros::NodeHandle& nh) : nh_(nh)
  {
    // Initialize BNO055 sensor driver.
    // Opens I2C Bus and locates BNO055 sensor.
    if (bno055_driver_.initI2c() < 0)
    {
      ROS_ERROR("Failed to initialize BNO055 Driver.");
    }
 
    // Initialize ROS publishers and ROS topics.
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 100);
    euler_pub_ = nh_.advertise<ros_bno055::OrientationEuler>("/imu/orientation_euler", 100);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/magnetic_field", 100);
    temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("/imu/temperature", 100);
    grv_pub_ = nh_.advertise<ros_bno055::Gravity>("/imu/gravity", 100);
  }

  // Method to set up BNO055 sensor.
  void start()
  {
    // Load calibration offsets and radii.
    if (bno055_driver_.loadCalib() < 0)
    {
      ROS_ERROR("Failed to load calibration offset and radius data.");
    }
    // Set operation mode to IMU.
    // Magnetometer disabled.
    if (bno055_driver_.setImuMode() < 0)
    {
      ROS_ERROR("Failed to set operation mode to IMU.");
    }
    // Set operation mode to NDOF.
    // Magnetometer enabled.
    /*if (bno055_driver_.setNdofMode() < 0)
    {
      ROS_ERROR("Failed to set operation mode to NDOF.");
    }*/
    if (bno055_driver_.getCalibStat() < 0)
    {
      ROS_ERROR("Failed to get calibration status data.");
    }
  }
  /*
  // Method to check if BNO055 sensor is calibrated.
  bool isCalibrated()
  {
    // If operation mode set to NDOF, ensure system is fully calibrated.
    if (bno055_driver_.getOprMode() == OprMode::NDOF && bno055_driver_.data_.calib_stat_sys_ == 3)
    {
      return true;
    }
    // If operation mode set to IMU, 
    // ensure accelerometer and gyroscope are fully calibrated.
    else if (bno055_driver_.getOprMode() == OprMode::IMU && bno055_driver_.data_.calib_stat_acc_ == 3 && bno055_driver_.data_.calib_stat_gyr_ == 3)
    {
      return true;
    }
    else
    {
      return false;
    } 
  }
  */
  void publishData()
  {
    // Get necessary sensor data.
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
    if (bno055_driver_.getLia() < 0)
    {
      ROS_ERROR("Failed to get linear acceleration data.");
    }
    if (bno055_driver_.getGrv() < 0)
    {
      ROS_ERROR("Failed to get gravity vector data.");
    }
    if (bno055_driver_.getTemp() < 0)
    {
      ROS_ERROR("Failed to get temperature data.");
    }

    // Construct ROS messages.
    ros::Time time_stamp = ros::Time::now();

    imu_msg_.header.stamp = time_stamp;
    imu_msg_.header.frame_id = "imu_link";
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
    //imu_msg_.linear_acceleration.x = bno055_driver_.data_.lia_x_;
    //imu_msg_.linear_acceleration.y = bno055_driver_.data_.lia_y_;
    //imu_msg_.linear_acceleration.z = bno055_driver_.data_.lia_z_;
    
    mag_msg_.header.stamp = time_stamp;
    mag_msg_.magnetic_field.x = bno055_driver_.data_.mag_x_;
    mag_msg_.magnetic_field.y = bno055_driver_.data_.mag_y_;
    mag_msg_.magnetic_field.z = bno055_driver_.data_.mag_z_;

    temp_msg_.header.stamp = time_stamp;
    temp_msg_.temperature = bno055_driver_.data_.temp_;

    euler_msg_.header.stamp = time_stamp;
    euler_msg_.heading = bno055_driver_.data_.eul_heading_;
    euler_msg_.roll = bno055_driver_.data_.eul_roll_;
    euler_msg_.pitch = bno055_driver_.data_.eul_pitch_;

    grv_msg_.header.stamp = time_stamp;
    grv_msg_.x = bno055_driver_.data_.grv_x_;
    grv_msg_.y = bno055_driver_.data_.grv_y_;
    grv_msg_.z = bno055_driver_.data_.grv_z_;
    
    // Publishes ROS messages to ROS topics.
    imu_pub_.publish(imu_msg_);
    mag_pub_.publish(mag_msg_);
    temp_pub_.publish(temp_msg_);
    euler_pub_.publish(euler_msg_);
    grv_pub_.publish(grv_msg_);
  }

  ~Bno055Node()
  {}
private:
  /* ROS */
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher temp_pub_;
  ros::Publisher euler_pub_;
  ros::Publisher grv_pub_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::MagneticField mag_msg_;
  sensor_msgs::Temperature temp_msg_;
  ros_bno055::OrientationEuler euler_msg_;
  ros_bno055::Gravity grv_msg_;
  /* BNO055 */
  bno055::Bno055Driver bno055_driver_;
};
} // namespace bno055

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bno055_node");
  ros::NodeHandle nh;
  bno055::Bno055Node bno055_node(nh);

  ros::Rate loop_rate(100);
   
  bno055_node.start();

  while (ros::ok())
  {
    bno055_node.publishData();
      
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
