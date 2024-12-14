# f1tenth

to run:

install docker.
get the foxy distro of ros2 running.
bind the container to the host folder
navigate to the ws directory.

source /opt/ros/foxy/setup.bash
source /lab1_ws/install/setup.bash
colcon build
ros2 launch lab1_pkg mylaunch.py

this should work, i really cant help much if it doesnt lol