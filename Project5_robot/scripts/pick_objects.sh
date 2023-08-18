#!/bin/sh

# catkin workspace path
PATH_CATKIN_WS="/home/robond/workspace/catkin_ws" 

xterm  -e  " source ${PATH_CATKIN_WS}/devel/setup.bash; roscore" & 
sleep 5
xterm  -e  " source ${PATH_CATKIN_WS}/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/slam_test.world" &
sleep 5 
xterm  -e  " source ${PATH_CATKIN_WS}/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${PATH_CATKIN_WS}/src/map/map.yaml " &
sleep 5
xterm  -e  " source ${PATH_CATKIN_WS}/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " source ${PATH_CATKIN_WS}/devel/setup.bash; roslaunch pick_objects pick_objects.launch "
