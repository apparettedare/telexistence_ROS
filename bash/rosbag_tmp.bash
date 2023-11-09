#!/bin/bash

rosbag_file="rosbag_controller"

if [ $# = 0 ]; then
    echo "rosbag file name = " $rosbag_file
else
    echo "rosbag file name = " $1
    rosbag_file=$1
fi

cd ~/catkin_ws/src/telexistence/bash
rosbag record   /oculus_3d/  \
                -o $rosbag_file
