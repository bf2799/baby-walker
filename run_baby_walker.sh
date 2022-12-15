#!/bin/sh
cd /home/pi/catkin_ws
sudo rm -rf build_isolated/baby_walker
sudo rm -rf devel_isolated/baby_walker
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg baby_walker -j2
roslaunch /home/pi/catkin_ws/src/baby_walker/launch/baby_walker.launch
