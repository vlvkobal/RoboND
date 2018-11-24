#!/bin/sh
CATKIN_WS="$(cd $(dirname "$0")/../..> /dev/null && pwd)"
xterm -e "ROBOT_INITIAL_POSE='-x -4.0 -y 6.0 -Y 4.712' roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CATKIN_WS/src/World/U-world.world" &
sleep 12
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=$CATKIN_WS/src/turtlebot_simulator/turtlebot_gazebo/launch/includes/gmapping.launch.xml" &
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm -e "rosrun wall_follower wall_follower"