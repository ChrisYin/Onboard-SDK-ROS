#!/bin/bash
source /home/ubuntu/M100_RCphone/devel/setup.bash
sleep 1
roscore&
sleep 12
roslaunch rosbridge_server rosbridge_websocket.launch&
sleep 5
roslaunch dji_sdk sdk_manifold.launch
