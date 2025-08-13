# 🦜 Parrot Robot - ROS 2 Jazzy

A social parrot robot built with ROS 2 and EduBlocks (Blockly-style interface for kids).  
Tested on Raspberry Pi using ROS 2 Jazzy.

## Features
- Head nodding, wing flapping, sound playback
- Vision (YOLO) and GPIO / HTTP bridge for Blockly integration
- One-package ROS 2 workspace

## Quickstart
```bash
cd ~/parrot_ws
colcon build
source install/setup.bash
ros2 launch parrot_robot bringup.launch.py
