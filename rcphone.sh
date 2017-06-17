#!/bin/bash
source /home/ubuntu/chris/M100_RCphone/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 0.5 
rosrun rcphone listener
