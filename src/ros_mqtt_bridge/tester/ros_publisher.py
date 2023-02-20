#!/usr/bin/env python3

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from threading import Thread, Timer
import time
import numpy as np
from termcolor import colored, cprint
from collections import defaultdict
import json
import os
from datetime import datetime
from importlib import import_module

class listener(Thread):
    def __init__(self, wait=True):
        Thread.__init__(self)
        self.freq_list  = list()
        self.freq_list_ros = list()
        self.color = 'green'
        self.count = 0
        self.rate = rospy.Rate(2000)
        self._stop = False
        self.wait = wait
        self.expected_hz = np.arange(100,1100,100)
        self.round       = 1
        self.data = defaultdict(lambda: defaultdict(list))

        self.subscribe_manual()
        # rospy.Subscriber('chatter', Int32MultiArray, self.callback)
        self._timer = Timer(3.0, self.waiting_message)
        # self._timer.start()

    def subscribe_manual(self):
        self.path = os.getcwd()
        self.topic = json.load(open(self.path + '/src/ros_mqtt_bridge/topic.json'))
        [_topic_list, _msg_name_list] = self.topic.keys()
        _topic      = self.topic[_topic_list]
        _msg_name   = self.topic[_msg_name_list]
        for i in range(len(_topic)):
            _msg_type = self._get_message_class(_msg_name[i])
            rospy.Subscriber(_topic[i], _msg_type, self.callback, _topic[i])
            print(f'            {_topic[i]} : message {_msg_name[i]}')

    def _get_message_class(self, classname):
        msg_lib, msg_type = classname.split('/')
        return getattr(import_module(msg_lib+'.msg'), msg_type)
    
    def get_frequency(self):
        self.delta_time_ros     = self.curr_time_ros - self.prv_time_ros

        self.freq_list_ros.append(1/self.delta_time_ros)

        # message = 'Sampling rate : ros is '+ str(round(self.freq_list_ros[-1],4)) +'Hz , in time function is '+str(round(self.freq_list[-1],4))+' hz '

        # rospy.loginfo(message)
        self.prv_time_ros = self.curr_time_ros

    def new_frequency(self):
        print(colored('{listener}', self.color))
        print('Summary is ros time :', self.get_average_freq('ros'))
        print('Length is ',self.count)
        print(len(self.freq_list_ros))
        self.count = 0
        self.freq_list_ros = list()
        self.round = self.round + 1
    
    def waiting_message(self):
        print(colored('{listener}', self.color) + ' Waiting for message ...')
    
    def callback(self, data, topic):
        if self.count == 0:
            print(colored('{listener}', self.color) + ' Received message ...')
            self.prv_time_ros = rospy.get_time()
            self.count = 1
            self.data[topic]['ts'].append(self.prv_time_ros)
            # self.data[str(self.expected_hz[self.round-1])]['ts'].append(self.prv_time_ros)
            
        else:
            if data.data == 'Stop':
                self._stop = True
            else:
                self.curr_time_ros  = rospy.get_time()
                self.get_frequency()
                self.count = self.count + 1
            self.data[topic]['ts'].append(self.curr_time_ros)
            # self.data[str(self.expected_hz[self.round-1])]['ts'].append(self.curr_time_ros)
        
        # self.data[str(self.expected_hz[self.round-1])]['data'].append(data.data)
        
        self.data[topic]['data'].append(data.data)
        self._timer.cancel()

        self._timer = Timer(8.0, self.new_frequency)
        self._timer.start()
    
    def get_average_freq(self, type_time):
        if type_time == 'ros':
            return sum(self.freq_list_ros)/len(self.freq_list_ros)

    def stop_thread(self, flag):
        self._stop = flag
    
    def run(self):
        while True:
            if self._stop:
                break
            self.rate.sleep()
        print(os.getcwd())
        name = str(len(self.topic['topic'])) + '_ros_test.json'
        print('Filename is {}. Waiting for recording process ...'.format(name))
        with open(name, 'w') as json_file:
        	json.dump(self.data, json_file)

class talker(Thread):
    def __init__(self, rate, subscriber=None):
        Thread.__init__(self)
        self.rate = rate
        self.pub = rospy.Publisher('chatter', Int32MultiArray, queue_size=1)
        self.subscriber = subscriber
        
        self.ros_rate = rospy.Rate(int(rate)) # 10hz
        self.real_rate = list()
        
        self.init = 0
        self.color = 'yellow'
    
    def run(self):
        print(colored('{talker}', self.color) + ' Start to publish ...')
        
        while True:
            if self.init == 0:
                prv_time = rospy.get_time()
                start_time = rospy.get_time()
                self.init = 1
                self.count = 1
            else:
                now = rospy.get_time()
                if now - start_time > 20:
                    if self.rate == '1000':
                        hello_str = 'Stop'
                        self.pub.publish(hello_str)
                    break
                else:
                    # hello_str = "hello world : " + str(round((1/(now-prv_time)),4)) + ' Hz'
                    hello_str = np.array([100,100,100,100,100,100])
                    hello_str = Int32MultiArray(data=hello_str)
                    self.real_rate.append(1/(now-prv_time))
                    # rospy.loginfo(hello_str)
                    prv_time = now
                    self.pub.publish(hello_str)
                    self.ros_rate.sleep()
                    self.count = self.count + 1
            # time.sleep(ts)
        print(colored('{talker}', self.color))
        print('Average of publish rate is : ',sum(self.real_rate[1:])/len(self.real_rate[1:]))
        print('Length is ',self.count)
        

if __name__ == '__main__':
    print('Would like to run Publisher / Subscriber or Both ?')
    print('     [p] : Publisher')
    print('     [s] : Subscriber')
    print('     [b] : Both')
    c = input('')
    if c == 'p' or c == 'P':
        try:
            rospy.init_node('Publisher', anonymous=True)
            rate = np.arange(100,1100,100)
            for r in rate:
                print('\nPublishing rate is {} Hz\n'.format(r))

                publisher = talker(r)
                publisher.start()
                publisher.join()

                time.sleep(8)
                print('\n#############################################################')
                del publisher
                time.sleep(2)
        except KeyboardInterrupt:
            pass
    elif c == 's' or c == 'S':
        rospy.init_node('Subscriber', anonymous=True)
        subscriber = listener(wait=True)
        subscriber.start()
        st = input('')
        if st == 'q':
            subscriber.stop_thread(True)
    elif c == 'b' or c == 'B':
        try:
            rospy.init_node('test_rate', anonymous=True)
            rate = np.arange(100, 1100, 100)
            subscriber = listener(wait=True)
            subscriber.start()
            for r in rate:
                # print('\nPublish rate is :',r)
                print('\nPublishing rate is {} Hz\n'.format(r))
                
                publisher = talker(r, subscriber)
                publisher.start()
                publisher.join()

                time.sleep(10)
                print('\n######################## END #####################################')
                del publisher
                time.sleep(2)
            del subscriber
        except KeyboardInterrupt:
            pass
    else:
        print('Wrong letter !')

