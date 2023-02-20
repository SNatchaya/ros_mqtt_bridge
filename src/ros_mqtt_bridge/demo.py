#!/usr/bin/env python3

import subprocess, os, time
import rospy
import json
import sys
from ros_mqtt_bridge.bridge import *

class Demo():
    def __init__(self):
        self.path = os.getcwd()
        print(self.path)
        
    def _run_roscore(self):
        if self.roscore_chr == 'y':
            #subprocess.Popen(["gnome-terminal", "--", "bash", "roscore.sh"], cwd='/scr')
            subprocess.Popen(["gnome-terminal", "--", "roscore"])
    
    def _run_mosquitto(self):
        if self.mqtt_chr == 'y':
            # Run MQTT broker in localhost (your computer)
            subprocess.Popen(["gnome-terminal", "--", "mosquitto", "-v"])
            self.mqtt_host  = 'localhost'
        else:
            # Enter the MQTT host ip whenever the other computer is running the MQTT broker.
            self.mqtt_host  = input('Enter MQTT host ip: ')
    
    def start_demo(self):
        # Run `roscore` 
        # self.roscore_chr     = input('Would you like to run the `roscore` on your computer? (y/n) ')
        # self._run_roscore()

        # Condition if you would like to run the MQTT broker or not
        self.mqtt_chr        = input('Would you like to run the `Mosquitto` on your computer? (y/n) ')
        self._run_mosquitto()

        self.start_bridge()
    
    def start_bridge(self):

        # subprocess.Popen(["gnome-terminal", "--", "bash", "demo_script.sh"], cwd=self.path + '/scripts')
        # time.sleep(2)

        # Initial the bridge node namely 'bridge_node'
        rospy.init_node('bridge_node', anonymous=True)

        print('\n--------------------------------- Start the ROS-MQTT bridge demo ---------------------------------\n')
        print('Note : Press [q] and enter the enter button to stop the process, and press [Ctrl+C] in other terminal to stop the operation. \n')

        # In auto-detect case
        # self.t1 = RosToMqttBridge(host=self.mqtt_host)
        

        # In manual case
        
        self.t          = dict()
        # # Load the JSON file that contains the topics' names and message types of each topic.
        self.topic_ros  = json.load(open(self.path + '/src/ros_mqtt_bridge/topic2.json'))
        self.t1 = RosToMqttBridge(host=self.mqtt_host, topic=self.topic_ros)
        self.t1.start()

        # self.topic_mqtt = ['test1', 'test2', 'test3', 'test4', 'test5', 'test6']
        # self.t['Mqtt-to-Ros'] = MqttToRosBridge(host=self.mqtt_host, topic=self.topic_mqtt, ray_state=True)
        
        # for thread in list(self.t.keys()):
        #     self.t[thread].start()
      
    def stop_demo(self):
        # after stopping demo, it will stop bridge as well.
        
        self.t1.stop_actor()
        time.sleep(1)

        for i in self.topic_ros['topic']:
            print(f'{i}  : {self.t1.get_data(i)}')

        self.t1.kill_ray()
        
        # self.t1.kill_ray()
        sys.exit()

if __name__ == '__main__':
    demo = Demo()
    demo.start_demo()
    st = input('')
    if st == 'q':
        demo.stop_demo()