# Baby Walker - EECE5552

## Developer First Time Setup

1. Ensure 32-bit Raspbian Buster is installed on Raspberry Pi with Raspberry Pi Imager (https://www.raspberrypi.com/software/)
2. Ensure SSH is enabled on the Pi (https://raspberrypi-guide.github.io/networking/connecting-via-ssh#:~:text=By%20default%2C%20SSH%20is%20disabled,to%20SSH%20and%20click%20OK%20.)
3. SSH into the Pi by running `ssh pi@raspberrypi.local`
4. Ensure ROS Melodic is installed on Raspberry Pi. If not, follow these instructions: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi. When installing key, run `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -` if step 2.1 fails.
5. Build ROS workspace: `sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2`
6. Install python3 dependencies: `sudo apt-get install python3-yaml && sudo pip3 install rospkg catkin_pkg`
7. Clone baby walker repository into `~/catkin_ws/src`

## Developer Building and Running Package

1. Enter catkin_ws directory: `cd ~/catkin_ws`
2. Run `sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg baby-walker -j2`. If run multiple times a script or alias for this command may be useful.
3. Run `roscore`
4. Run launch file for baby walker package
