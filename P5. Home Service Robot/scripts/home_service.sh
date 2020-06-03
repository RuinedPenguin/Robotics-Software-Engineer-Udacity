#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/myrobot/catkin_ws/src/map/Laksh.world " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/robond/myrobot/catkin_ws/src/map/map.yaml " &
sleep 5
xterm -e " rosrun rviz rviz -d /home/robond/myrobot/catkin_ws/src/rvizConfig/home_service.rviz " &
sleep 10
xterm -e "rosrun add_markers add_markers " &
sleep 5
xterm -e "rosrun pick_objects pick_objects"
