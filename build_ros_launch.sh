#! /usr/bin/bash
colcon build && . install/setup.bash && ros2 launch simulator launcher.launch.py