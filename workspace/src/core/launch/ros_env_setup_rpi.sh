#!/bin/bash
# Настройки ROS для Raspberry Pi (ROS Master)

export ROS_MASTER_URI=http://192.168.1.151:11311
export ROS_IP=192.168.1.151

echo "ROS environment set for Raspberry Pi:"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
roslaunch core sensors_test.launch
