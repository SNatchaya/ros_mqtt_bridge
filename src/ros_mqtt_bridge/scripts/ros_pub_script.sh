#!/bin/bash
cd ..
cd tester
gnome-terminal --title 'ROS publisher' -e 'rosbag play rosbag_test.bag'