# Repository Contents

**auto_bot**  
This is a set up of a mobile robot (custom urdf) in a Gazebo (Harmonic) simulation using Nav2 in ROS 2 Humble. Nav2 has been used for autonomous navigation, avoiding static and dynamic obstacles while reaching a goal input by user.

**ROSArduinoBridge**  
This code turns an Arduino into a motor controller! It provides a simple serial interface to communicate with a high-level computer (e.g. running ROS), and generates the appropriate PWM signals for a motor driver, to drive two motors.

**diffdrive_arduino**  
This node is designed to provide an interface between a `diff_drive_controller` from `ros2_control` and an Arduino running firmware from `ros_arduino_bridge`.

**serial**  
This is a cross-platform library for interfacing with rs-232 serial like ports written in C++. It provides a modern C++ interface with a workflow designed to look and feel like PySerial, but with the speed and control provided by C++.


## Following is a sample showing the robot successfully navigating to the goal while avoiding obstacles

Drive Link:- https://drive.google.com/file/d/1PFR9V--mucsstYXugDUwDcRx7oSW2oQ6/view?usp=sharing


## Approach to localization, planning, and obstacle avoidance

1. A customized bot with dimensions 300cm*300cm*150cm was made with 50 cm radius wheels and 50 cm caster wheel

2. Gazebo plugins libgazebo_ros_diff_drive.so for differential drive, libgazebo_ros_ray_sensor.so for lidar and libgazebo_ros_camera.so for camera was used

3. Nav2 documentation was used for localization and mapping using SLAM and AMCL

4. Used simple commander api documentation and its example to understand and make script for user given input static and dynamic obstacle avoidance to rach goal

## Challenges faced and solutions

1. Challenge: ROS convention and camera convention for frame are different  
  Solution: Added a camera_link_optical that to compensate this with rpy="${-pi/2} 0 ${-pi/2}"

2. Challenge: Camera was obtaining images of different color  
  Solution: `<format>R8G8B8</format>` changed the format by hit and trial to get it right

3. Challenge: Map not subscribed on rviz even when topic rviz was published  
  Solution: Put map as fixed frame manually and set topic->Durability to Transient Local

4. Challenge: While using the script the bot sometimes would reach right orientation and sometimes not  
  Solution: Learnt that the orientations are in quaternion and got to know how the orientations are represented by them

