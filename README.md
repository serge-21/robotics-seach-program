# robotics-seach-program

## Overview
This program uses ROS, ROS smach, and YOLO to search for cake in the empty_stage_single_robot floor plan in the rosplan_stage_demo package. 
Once found the robot will stop the search. Every couple of seconds the robot reports progress (this includes everything it saw up to now).

Prerequisits:
- linux ubuntu 20.04
- ROS, and ROS smach (I installed it using [apptainer](https://apptainer.org/docs/admin/main/installation.html) -- recommended)
- YOLO
- rosplan_stage_demo package

---
NOTE: the package is called second_coursework, you may change that if you wish.

Before you start remember to:
- Change `coco.data` inside the config folder to point to the training set for YOLO
- Run the contianer using apptainer:
  - `apptainer run ros-container.sif`
- Make the folder available to ROS:
  - `source devel/setup.bash`
- Run roscre:
  - `roscore &`

To run the program just run it like a normal ROS program:
- In the terminal run the following to start both services:
  - `rosrun second_coursework action_client.py`
  - `rosrun second_coursework room_service.py`
- You may be using either bags or an actual camera attached to a robot. Please note: that the camera topic in the robot (and therefore, in the ROS bags) on which the
images are published is `/camera/image` instead of the `/usb_cam/image_raw` 

NOTE: the floor plan in `scripts/room_service.py` is taken from empty_stage_single_robot in the rosplan_stage_demo package. Once installed you can use the following to view the floor plan
```
$ roslaunch rosplan_stage_demo empty_stage_single_robot.launch
$ rosrun rviz rviz -d `rospack find rosplan_stage_demo`/config/rosplan_stage_demo.rviz
```
The floor plan is meant to look something like this:

![image](https://github.com/serge-21/robotics-seach-program/assets/26350015/68178a94-9914-4aff-a845-0f0b75741e6e)
