#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find World)/u_home.world
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find World)/map.yaml

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm  -e  "roslaunch Launch rviz_add_markers.launch" &
sleep 5
xterm  -e  "rosrun add_markers add_markers_node"
