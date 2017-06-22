#!/bin/bash
source /home/ubuntu/M100_RCphone/src/Onboard-SDK-ROS/rcphone.sh&
sleep 20
rosrun dji_sdk_demo dji_sdk_rctest1
