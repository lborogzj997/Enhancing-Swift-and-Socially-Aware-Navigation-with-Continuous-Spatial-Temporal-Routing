# Enhancing Swift and Socially-Aware Navigation with Continuous Spatial-Temporal Routing
Algorithm demo &amp; Guidelines for Simulating Mobile Robots in Crowded Environment

![image](https://github.com/user-attachments/assets/908d6e9e-1cac-4690-8727-24b935bdad52)


- Click to view the project demonstration video: https://www.youtube.com/watch?v=wsg-De5rDfc&t=54s

## This repository contains following items for the spatial-temporal routing project:
- MATLAB demo for the spatial-temporal routing algorithm
- Guidelines on how to implement the high-level path planning algorithm into a 3D pedestrian simulator using ROS packages
- A custom differential drive robotic wheelchair model in URDF format
- XML-based scene (generated from THÖR datasets: real human trajectories for human-robot interaction testing)
- MAT-based scene (artificially generated for testing robot navigation in large crowds)

## Software Requirements:
### System:
- Ubuntu 16.04.7
- [ROS Kinetic] (http://wiki.ros.org/kinetic/Installation/Ubuntu) (may work with other versions, tested and deployed on Kinetic)

## Require Packages:
- ROS Packages: https://github.com/srl-freiburg/pedsim_ros
- Matlab Packages: https://web.casadi.org/get/

## Installation

### Step 1
Set Up the Testing Environment in Your Workspace (SPACiS)

```cd [workspace]/src
git clone https://github.com/maprdhm/Spaciss.git  
cd Spaciss
git submodule update --init --recursive
cd ../..
catkin_make or catkin build (twice at the first time)
```

### Step 2
- Download the robot_wheelchair file into your workspace (catkin_ws/src)
- Replace the experimental_package file from the Pedestrian_simulator file

## Example usage
- launch the robot in the crowded environment:
```roslaunch experimental_package Scenario_test.launch```

- To change the environment in the launch file:
```<arg name="scene_file" value="$(find experimental_package)scenarios/business_area/low/thormap.xml"/>```

THÖR scenario: (Pedestrian_simulator/experimental_package/scenarios/business_area/low/thormap.xml)

## Data communication
- The position of simulated pedestrians can be extracted via the ROS topic: /pedsim_visualizer/tracked_persons (not a regular pose message; requires custom processing in MATLAB: MATLAB ROS Custom Messages)
- Alternatively, the position of simulated pedestrians can be extracted using the top-view camera in RVIZ:

    Set the color of the crowd to blue in the top-view camera
    Record the video in MP4 format
    Use Support\Crowd(blue)_tracking_fixed_frame.m to track the crowd's motion
  
- 'MotionControl_ros_robot.m' is a motion controller that, given the reference path, continuously publishes cmd_vel commands to control the robot and follow the path.






