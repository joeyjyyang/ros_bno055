# ros_bno055
**A ROS package containing a Linux hardware driver for Adafruit's BNO055 9-DoF Orientation Sensor Breakout and a ROS node wrapper that exposes the sensor driver to the ROS ecosystem.**

## Overview
The hardware driver for the BNO055 sensor is ROS-agnostic, meaning it provides a clean functional Linux interface to the sensor that can be used on any Linux system independent of ROS. The driver utilizes System Management Bus (SMBus), which is derived from the I2C serial bus protocol, for communication between the sensor and the OS. Additionally, the ROS node simply wraps around the driver and connects the sensor data to ROS topics.

## Prerequisites
### Software
- Ubuntu 16.04 (Xenial)
- ROS Kinetic
### Hardware
- Adafruit's BNO055 Absolute Orientation Sensor

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_bno055.git
cd .. 
sudo apt-get install -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make # catkin build ros_bno055 (if using catkin_tools)
source devel/setup.bash
rospack profile
```

## Setup
### Calibration
- BNO055 datasheet, including calibration instructions: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

## Nodes
- `bno055_node`

## Topics
- `/imu`
- `/magnetic_field`
- `/temperature`

## Messages
- `sensor_msgs/Imu`
- `sensor_msgs/MagneticField`
- `sensor_msgs/Temperature`

## Usage
### Example
```
roslaunch ros_bno055 bno055_imu.launch
```

## Notes
### Raspberry Pi I2C Clock Stretching (Bug)
Adafruit's BNO055 sensor utilizes I2C clock stretching to function correctly; however, there is an existing hardware bug within the Raspberry Pi (all versions and models) that prevents the hardware I2C from supporting the clock stretching functionality. As a result, there are 2 possible workarounds to choose from:
1. Enabling and using the software I2C driver on the Raspberry Pi, as it does support I2C clock stretching: https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md
- Note: if using method 1. above (enabling and using the software I2C driver on the Raspberry Pi), ensure that the `I2C_BUS` constant in `src/bno_driver.cpp` is set to `/dev/i2c-3`, instead of the default `/dev/i2c-1`.

2. Slowing down the I2C clock on the Raspberry Pi by reducing the baudrate: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang
