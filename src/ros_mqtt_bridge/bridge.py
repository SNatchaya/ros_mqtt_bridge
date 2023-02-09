#!/usr/bin/env python3

from threading import Thread, Timer
import paho.mqtt.client as mqtt
from importlib import import_module
import json, itertools
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from termcolor import colored, cprint
from pyfiglet import Figlet
import rospy
from datetime import datetime

class RosToMqttBridge(Thread):
    def __init__(self, host, topic=None):
        Thread.__init__(self)

        # Build the connection to MQTT broker
        self._stop_thread = False
        self.host       = host
        self.port       = 1883
        self.qos        = 0
        self.color      = 'cyan'
        self._topic_dict     = topic
        self.exo_graph_status = False
        self.data       = dict()
        self.ros_rate   = rospy.Rate(1000)
        self.threads    = dict()
        self.dummy      = dict()
        # self.main_state = False
        
        print('______________________________________Start______________________________________\n')
        self.f = Figlet(font='slant')
        print(colored(self.f.renderText('ROS-MQTT bridge'), self.color))

        self.client                 = mqtt.Client('Ros-to-Mqtt')
        self.client.connect(self.host, self.port)

        self._ros_timer = Timer(8.0, self._start_ros)
        self._ros_timer.start()

        print('ROS topic(s) :')
        if self._topic_dict == None:
            print('\n Use auto-detect the ROS topics\' name and message types\n')
            tp_list = rospy.get_published_topics()
            for topics, msg_name in tp_list:
                msg_pkg, msg_type_name = msg_name.split('/')
                if msg_type_name != 'Log' and msg_type_name != 'Clock':
                    sub_topic = topics.split('/')
                    msg_type = getattr(import_module(msg_pkg + '.msg'), msg_type_name)
                    rospy.Subscriber(topics, msg_type, self.ros_callback, topics)
                    print(f'            {topics} : message {msg_type_name}')
            print('\n')
        else:
            # Defind topic yourself
            # for x in ['pub', 'sub']:
            #     topic_lst       = self._topic_dict[x]['topic']
            #     msg_name_lst    = self._topic_dict[x]['message_type']
            #     if x == 'pub':
            #         title = '   Publish topic(s) :'
            #     else:
            #         title = '   Subscribe topic(s) :'
            #     print(title)
            #     self.threads[x]  = TransferThread(topic_lst, msg_name_lst, self.ros_callback)
            # print('\n')

            [_topic_list, _msg_name_list] = self._topic_dict.keys()
            _topic      = self._topic_dict[_topic_list]
            _msg_name   = self._topic_dict[_msg_name_list]
            for i in range(len(_topic)):
                _msg_type = self._get_message_class(_msg_name[i])
                rospy.Subscriber(_topic[i], _msg_type, self.ros_callback, _topic[i])
                print(f'            {_topic[i]} : message {_msg_name[i]}')
    
    def _update_data(self, topic, message):
        self.data[topic] = message

    def _get_message_class(self, classname):
        return getattr(import_module('std_msgs.msg'), classname)
    
    def get_data(self, topic):
        return self.data[topic]

    def set_main(self, main):
        self.main = main

    def search(self, lst, condition):
        for i in lst:
            if i == condition:
                return True
        return False
    
    def add_topic(self, topic, message_type):
        if self.search(self._topic_dict['topic'], topic):
            pass
        else:
            classname = self._get_message_class(message_type)
            rospy.Subscriber(topic, classname, self.ros_callback, topic)
            print(colored('{Ros-to-Mqtt}', self.color) + colored(' Add topic -> ', 'red') + topic + ' : message ' + message_type)
            self._topic_dict['topic'].append(topic)
            self._topic_dict['message_type'].append(message_type)    

    def expand_imu(self, imu, ts):
        imu_dict = self.create_dict()
        # imu_dict['Cheader']['seq'] = imu.cheader.seq
        # imu_dict['Cheader']['stamp']['secs'] = imu.cheader.stamp.secs
        # imu_dict['Cheader']['stamp']['nsecs'] = imu.cheader.stamp.nsecs
        # imu_dict['Cheader']['frame_id'] = imu.cheader.frame_id
        imu_dict['ts'] = ts
        imu_dict['data']['orient']['x']     = imu.orientation.x
        imu_dict['data']['orient']['y']     = imu.orientation.y
        imu_dict['data']['orient']['z']     = imu.orientation.z
        imu_dict['data']['orient']['w']     = imu.orientation.w

        imu_dict['data']['orientation_covarience']  = imu.orientation_covariance

        imu_dict['data']['angular_velocity']['x']   = imu.angular_velocity.x
        imu_dict['data']['angular_velocity']['y']   = imu.angular_velocity.y
        imu_dict['data']['angular_velocity']['z']   = imu.angular_velocity.z

        imu_dict['data']['angular_velocity_covarience'] = imu.angular_velocity_covariance

        imu_dict['data']['linear_acceleration_covarience'] = imu.linear_acceleration_covariance
        imu_dict['data']['linear_acceleration']['x']       = imu.linear_acceleration.x
        imu_dict['data']['linear_acceleration']['y']       = imu.linear_acceleration.y
        imu_dict['data']['linear_acceleration']['z']       = imu.linear_acceleration.z
        return imu_dict
    
    def ros_callback(self, message, topic):

        # Change the message type of ROS
        ts          = str(datetime.now().timestamp())
        
        # Find sub topic
        topic_list  = topic.split('/')
        # ts          = str(datetime.now().strftime('%H:%M:%S.%f').replace('.',':'))    
        key         = topic_list[-1]
        if self.search(topic_list, 'imu'):
            ros_message = self.expand_imu(message, ts)
            data = ros_message
        else:
            ros_message = message.data
            data = {'ts': ts, key : ros_message}
        
        # update log data
        self._update_data(topic, ros_message)

        data = json.dumps(data)
        self.client.publish(topic, data, self.qos)
        
        self._ros_timer.cancel()

        self._ros_timer = Timer(5.0, self._finished_from_ros)
        self._ros_timer.start()
    
    def disconnect(self):
        self.client.disconnect()
        self.client.loop_stop()
        self._ros_timer.cancel()

        print(colored('{Ros-to-Mqtt} Client disconnected', 'red'))
        self._stop_thread = True
    
    def rospy_shutdown(self):
        rospy.on_shutdown(self._rospy_shutdown)
        rospy.core.signal_shutdown('__Stop rospy__')
    
    def _rospy_shutdown(self):
        # print(self.ros_msg)
        print('_______________________________Shutdown ros node_______________________________')

    def _finished_from_ros(self):
        print(colored('{Ros-to-Mqtt}', self.color) + ' Transfer Finished : Received message from ROS and waiting for new message ...')

    def _start_ros(self):
        print(colored('{Ros-to-Mqtt}', self.color) + ' Waiting for new message ...')
    
    def run(self):
        self.client.loop_start()
        while True:
            if self._stop_thread:
                break
            self.ros_rate.sleep()

class MqttToRosBridge(Thread):
    def __init__(self, host, topic):
        Thread.__init__(self)
        # self.host       = '192.168.1.5'

        self._stop_thread = False
        self.host       = host
        self.port       = 1883
        self.color      = 'yellow'
        self.stop_thread = False
        self.topic      = topic
        self.data       = dict()

        print('______________________________________Start_____________________________________\n')
        self.f = Figlet(font='slant')
        print(colored(self.f.renderText('MQTT-ROS bridge'), self.color))


        self.client                 = mqtt.Client('Mqtt-to-Ros')
        self.client.connect(self.host, self.port)
        self.client.on_message      = self.on_message
        print('MQTT topic(s) :')
        for i in self.topic:
            self.client.subscribe(i)
            print(f'            {i}')
        print('\n')

        self._mqtt_timer            = Timer(5.0, self._start_mqtt)
        self._mqtt_timer.start()
    
    def _update_data(self, topic, message):
        self.data[topic] = message

    def get_data(self, topic):
        return self.data[topic]

    def set_main(self, main):
        self.main = main
        # self.main_state = True
    
    def search(self, lst, condition):
        for i in lst:
            if i == condition:
                return True
        return False
    
    def on_message(self, client, userdata, msg):
        # Stop timer when received the message

        
        mqtt_message = msg.payload.decode()
        # self.main.get_class('_exo_command').get_message_from_bridge(topic=msg.topic, _message=mqtt_message)

        # check message is string, integer or float
        if mqtt_message.isdigit():
            rospy.Publisher(msg.topic, Int32, queue_size=1).publish(int(mqtt_message))
        else:
            # find symbol instead of charactors
            if mqtt_message.find('[') == 0 or mqtt_message.find('(') == 0 or mqtt_message.find(']') == 0 or mqtt_message.find(')') == 0:
                mqtt_decode = str(msg.payload.decode()) # str('x.json')
                mqtt_message = json.loads(mqtt_decode)
                mqtt_message = np.array(mqtt_message)
                if type(mqtt_message[0]) == int:
                    rospy.Publisher(msg.topic, Int32MultiArray, queue_size=1).publish(Int32MultiArray(data=mqtt_message))   
                elif type(mqtt_message[0]) == float:
                    rospy.Publisher(msg.topic, Float32MultiArray, queue_size=100).publish(mqtt_message)
            else:
                rospy.Publisher(msg.topic, String, queue_size=1).publish(mqtt_message)

        
        self._update_data(msg.topic, mqtt_message)
        self._mqtt_timer.cancel()
        # # Start timer again
        self._mqtt_timer            = Timer(2.0, self._finished_from_mqtt)
        self._mqtt_timer.start()
    
    def disconnect(self):
        self.client.disconnect()
        self.client.loop_stop()
        self._mqtt_timer.cancel()
        print(colored('{Mqtt-to-Ros} Client disconnected', 'red'))

        self._stop_thread = True
    
    def _finished_from_mqtt(self):
        print(colored('{Mqtt-to-Ros}', self.color) + ' Transfer Finished : Received message from MQTT and waiting for new message ...')

    def _start_mqtt(self):
        print(colored('{Mqtt-to-Ros}', self.color) + ' Waiting for new message ...')

    def run(self):
        self.client.loop_start()
        ros_rate = rospy.Rate(1000)

        while True:
            if self._stop_thread:
                break
            ros_rate.sleep()
