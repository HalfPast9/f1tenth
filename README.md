# f1tenth

to run:

install docker.
get the foxy distro of ros2 running.
bind the container to the host folder
navigate to the ws directory.

some useful commands:

connecting to your docker container (assuming it is created and running):
docker exec -it <container_name_or_id> /bin/bash

to run a new build:
source /opt/ros/foxy/setup.bash
source /lab1_ws/install/setup.bash
colcon build
ros2 launch lab1_pkg mylaunch.py


consider downloading rqt_graph, u can visualise the ocnnections to make sure they work the way they were coded.

TO INSTALL RQT_GRAPH:
on windows: install vcxsrv.
start it up, enable "disable access control" option
leave display number as -1

this should work, i really cant help much if it doesnt lol