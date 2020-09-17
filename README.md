# bno055_driver
**A ROS package containing a Linux hardware driver for Adafruit's BNO055 9-DoF Orientation Sensor Breakout and a ROS node wrapper that exposes the sensor driver to the ROS ecosystem.**

## Overview
The hardware driver for the sensor is ROS-agnostic, meaning it provides a clean functional Linux interface to the sensor that can be used on any Linux system independent of ROS. The driver utilizes System Management Bus (SMBus), which is derived from the I2C serial bus protocol, for communication between the BNO055 sensor and the OS. Additionally, the ROS node simply wraps around the driver and connects the sensor data to ROS topics.

## Prerequisites
### Software
- Ubiquity Robotics Raspberry Pi Image: https://downloads.ubiquityrobotics.com/pi.html
	- Ubuntu 16.04 (Xenial)
	- ROS Kinetic
### Hardware
- Adafruit's BNO055 Absolute Orientation Sensor: https://www.adafruit.com/product/2472

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/bno055_driver.git
cd .. 
sudo apt-get install -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make # catkin build bno055_driver (if using catkin_tools)
source devel/setup.bash
rospack profile
```

## Nodes
- `bno055_node`

## Topics
- `/bno055_node/imu`
- `/bno055_node/magnetic_field`
- `/bno055_node/temperature`

## Messages
- `sensor_msgs/Imu`
- `sensor_msgs/MagneticField`
- `sensor_msgs/Temperature`

## Usage
### Example
```
roslaunch bno055_driver bno055_imu.launch
```

## Notes
### Raspberry Pi I2C Clock Stretching (Bug)
Adafruit's BNO055 sensor utilizes I2C clock stretching to function correctly; however, there is an existing hardware bug within the Raspberry Pi (all versions and models) that prevents the hardware I2C from supporting the clock stretching functionality. As a result, there are 2 possible workarounds to choose from:
1. Enabling and using the software I2C driver on the Raspberry Pi, as it does support I2C clock stretching: https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md
2. Slowing down the I2C clock on the Raspberry Pi by reducing the baudrate: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang
