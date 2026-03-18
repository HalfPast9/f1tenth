
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



Simple day-to-day flow:

1. **Start everything** — `docker compose up` from `f1tenth_gym_ros` dir on host
2. **Shell in** — `docker exec -it f1tenth_gym_ros-sim-1 /bin/bash` in a terminal
3. **Launch sim when testing** — `source /opt/ros/foxy/setup.bash && source install/local_setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
4. **Edit code** on host in your editor, changes reflect in container instantly
5. **Build in container** — `cd /lab2_ws && colcon build`
6. **Run your node** in a second `docker exec` terminal
7. **When done** — `docker compose down` to shut everything off cleanly (avoids the zombie process issue)

The browser at `http://localhost:8080/vnc.html` is just for watching RViz. Everything else happens in your terminals.



Host — mkdir -p ~/clonedrepos/f1tenth/lab3_ws/src/<package_name>
docker-compose.yml — add the mount: - /home/shvempat/clonedrepos/f1tenth/lab3_ws/src/<package_name>:/sim_ws/src/<package_name>
then rebuild container
cd /tmp
ros2 pkg create --build-type ament_python test_node
cp -r /tmp/test_node/. /sim_ws/src/test_node/




consider downloading rqt_graph, u can visualise the ocnnections to make sure they work the way they were coded.

TO INSTALL RQT_GRAPH:
on windows: install vcxsrv.
start it up, enable "disable access control" option
leave display number as -1

this should work, i really cant help much if it doesnt lol
