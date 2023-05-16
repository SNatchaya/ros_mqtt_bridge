#!/bin/bash
cd ..
cd tester
gnome-terminal --title 'ROS publisher' -e 'rosbag play rosbag_test.bag'
gnome-terminal --title 'MQTT publisher' -e 'python3 mqtt_pub.py'
sleep 1
gnome-terminal --title 'MQTT subscriber (subscribe topic from ROS pub)' -e 'mosquitto_sub -h localhost -t /exvis/jointAngle'
gnome-terminal --title 'ROS subscriber (subscribe topic from MQTT pub)' -e 'rostopic echo /sub'