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

### Step 1
Set up the testing envornment in your workspace (SPACiS)

```cd [workspace]/src
git clone https://github.com/maprdhm/Spaciss.git  
cd Spaciss
git submodule update --init --recursive
cd ../..
catkin_make or catkin build (twice at the first time)
```

### Step 2
-Dowland the robot_wheelchair file in your workspace (catkin_ws/src)
-Replace the experimental_package file from Pedestrian_simulator file

## Example usage
- launch the robot in the crowded environment:
```roslaunch experimental_package Scenario_test.launch```

- In launch file, change different environment here:
```<arg name="scene_file" value="$(find experimental_package)scenarios/business_area/low/thormap.xml"/>```

THÖR scenario: (Pedestrian_simulator/experimental_package/scenarios/business_area/low/thormap.xml)

## Communication
- The position of simulated pdestrian can be extracted by ROS topic: /pedsim_visualizer/tracked_persons (not a regular pose msg, need further custom process in matlab: https://uk.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html)
- Alternatively, the position of simulated pedesrian can be extracted by Top view camera in RVIZ: Top view camera -> Set color of crowd to blue -> record video in MP4 file -> feed to (Support\Crowd(blue)_tracking_fixed_frame.m) to track the crowd motion
- (MotionControl_ros_robot.m.m) is a motion controller, given the reference path, it keep publish cmd_vel comment to control robot follow the path.




