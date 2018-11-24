#!/bin/sh
CATKIN_WS="$(cd $(dirname "$0")/../..> /dev/null && pwd)"
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CATKIN_WS/src/World/U-world.world" &
sleep 12
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"