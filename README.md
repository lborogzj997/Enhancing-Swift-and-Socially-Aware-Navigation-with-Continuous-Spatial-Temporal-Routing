# Spatial-Temporal Routing for Robots in Dynamic Human Environments
Algorithm demo &amp; Guidelines for Simulating Mobile Robots in Crowded Environment

![image](https://github.com/user-attachments/assets/908d6e9e-1cac-4690-8727-24b935bdad52)


- Click to view the project demonstration video: https://www.youtube.com/watch?v=wsg-De5rDfc&t=54s

## This repository contains following items for the spatial-temporal routing project:
- Matlab demo for spatial-temporal routing algorithm
- Guidlines on how to impiment high-level path planning algorithm into 3D pedestrian simulator in ROS Packages
- A costom differential drive Robotic wheelchair model in urdf format.
- XML based scene (Generate from THÖR DATASETS: real human trajectoires for human-robot interaction testing purpose)
- MAT based scene (Generate artifically for testing robot navigation with large crowd size)


## Software Requirements:
### System:
- Ubuntu 16.04.7
- [ROS Kinetic] (http://wiki.ros.org/kinetic/Installation/Ubuntu) (may work with other versions, tested and deployed on Kinetic)

## Require Packages:
- ROS Packages: https://github.com/srl-freiburg/pedsim_ros
- Matlab Packages: https://web.casadi.org/get/

## Installation

Set up the testing envornment in your workspace

```cd [workspace]/src
git clone https://github.com/srl-freiburg/pedsim_ros.git  
cd pedsim_ros
git submodule update --init --recursive
cd ../..
catkin build -c  # or catkin_make

## Example usage
``` roslaunch experimental_package Scenario_test.launch
(Need download the robot urdf under src file)
