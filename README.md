# bno055_driver
**A ROS driver for Adafruit's BNO055 9-DoF Orientation Sensor Breakout.**

## Overview
Utilizes System Management Bus (SMBus), which is derived from the I2C serial bus protocol, for communication between the BNO055 sensor and the OS.
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

## Parameters

## Usage
### Example
```
roscore
rosrun bno055_driver bno055_node
```

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang
