#!/bin/bash
roscore &
sleep 5
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 5
roslaunch dji_sdk sdk_manifold.launch
