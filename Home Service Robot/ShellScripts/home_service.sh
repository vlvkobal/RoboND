#!/bin/sh
CATKIN_WS="$(cd $(dirname "$0")/../..> /dev/null && pwd)"
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CATKIN_WS/src/World/U-world.world" &
sleep 12
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$CATKIN_WS/src/World/my_map.yaml" &
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm -e "rosrun pick_objects pick_objects" &
xterm -e "rosrun add_markers add_markers"