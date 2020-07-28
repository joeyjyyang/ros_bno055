# bno055_driver
**A ROS driver for Adafruit's BNO055 9-DoF Orientation Sensor Breakout.**

## Overview

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
source ~/.bashrc # get email and text credentials
rospack profile
```

## Nodes

## Usage
### Example

