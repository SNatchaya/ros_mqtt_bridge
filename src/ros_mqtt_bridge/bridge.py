#!/usr/bin/env python3

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

# thread library
from threading import Thread, Timer
import ray

# mqtt library
import paho.mqtt.client as mqtt

# normal library
from importlib import import_module
import json
import numpy as np
from termcolor import colored
from pyfiglet import Figlet
from collections import defaultdict
from typing import *
import psutil

def search(lst, condition) -> bool:
    for i in lst:
        if i == condition:
            return True
    return False

@ray.remote(num_cpus=1)     
class _RosToMqttBridge_(object):
    def __init__(self, host:str, topic:str, message_pkg:str, message_lib:str, color:str, max_length:int):
        if topic == '/imu/data':
            name        = 'imuData'
        else:
            name = topic.split('/')[-1]
            
        node_name   = ('_').join([name, 'bridge_node'])
        client_name = ('_').join([name, 'bridge_client'])
        rospy.init_node(node_name, anonymous=True)
        # Mqtt variables
        self.host   = host
        self.PORT   = 1883

        self.color              = color
        self.max_length         = max_length
        
        # Create MQTT client and build connection
        self.client = mqtt.Client(client_name) 
        self.client.connect(self.host, self.PORT)
        
        # self.data           = defaultdict(lambda: defaultdict(list))
        # self.data           = self.create_dict()
        self.data           = defaultdict()
        self.count          = 0
        self.len_data       = 0

        # Ros variables
        self.ROS_RATE       = rospy.Rate(1000)
        self.topic          = topic
        self.message_type   = message_lib
        self.message_pkg    = message_pkg

        rospy.Subscriber(self.topic, self.message_pkg, self._ros_callback, self.topic)
    
    def create_dict(self) -> defaultdict:
        return defaultdict(self.create_dict)
    
    def get_data(self, topic:str) -> defaultdict:
        return self.data[topic]

    def _update_data(self, topic:str, message):
        self.data[topic] = message

    def _get_subattribute_names(self, obj:object, condition:bool=True) -> List[str]:
        """
        obj         : Main attribute
        condition   : If condition is True, the function name in below is ignored.
                        ['deserialize', 'deserialize_numpy', 'header', 'serialize', 'serialize_numpy']
        """
        if condition is not True:
            func_name = [func for func in dir(obj) if not func.startswith("__") and not func.startswith('_')]
        else:
            not_use             = ['deserialize', 'deserialize_numpy', 'header', 'serialize', 'serialize_numpy']
            func_name_default   = [func for func in dir(obj) if not func.startswith("__") and not func.startswith('_')]
            func_name           = []
            for i in func_name_default:
                if all(i != func for func in not_use):
                    func_name.append(i)
        return func_name
    
    def _sensor_message(self, message:object, ts) -> defaultdict:
        """
        message     : Attribute variable
                      For example :
                            - Imu
                            - Magneticfields
        ts          : Timestamp      
        """
        sensor_dict         = self.create_dict()
        sensor_dict['ts']   = ts
        
        func_name   = self._get_subattribute_names(message)
        for param in func_name:
            command = ('.').join(['message', param])
            axis    = self._get_subattribute_names(eval(command))
            if axis == []:
                sensor_dict['data'][param] = eval(command)
            else:
                for ax in axis:
                    if any(ax == p for p in ['count', 'index']):
                        continue
                    command     = ('.').join(['message', param, ax])
                    sensor_dict['data'][str(param)][ax] = eval(command)
        return sensor_dict
    
    def _ros_callback(self, message, topic:str):
        if self.count == 0:
            s = topic + ' '*(self.max_length - len(topic))
            print(s + ' : Receiving message') 
            self.count = 1
    
        # ts          = str(datetime.now().strftime('%H:%M:%S.%f').replace('.',':'))    
        ts = str(rospy.get_time())
        
        if self.message_type.split('/')[0] == 'sensor_msgs':
            ros_message = self._sensor_message(message, ts)
            data        = ros_message
        else:
            # For standard message
            key = topic.split('/')[-1]
            ros_message = message.data
            data = {'ts': ts, 'data': ros_message}
        
        # update log data
        # self._update_data(topic, ros_message)
        self.data[topic] = ros_message

        # For SI format
        data = json.dumps(data)
        self.client.publish(topic, data, 0)
        self.len_data += 1

        # Timer
        self.ros_timer.cancel()
        self.ros_timer = Timer(10.0, self._finished_from_ros)
        self.ros_timer.start()

    def _finished_from_ros(self):
        s = self.topic + ' '*(self.max_length - len(self.topic))
        print(s + f' : ({self.len_data} msgs) Transfer Finished from ROS and waiting for new message ...')
        self.count      = 0
        self.len_data   = 0

    def _start_ros(self):
        s = self.topic + ' '*(self.max_length - len(self.topic))
        print(s + ' : Waiting for message ...') 

    def stop_thread(self, flag):
        self.client.disconnect()
        # self.client.loop_stop()
        self.ros_timer.cancel()

        s = self.topic + ' '*(self.max_length - len(self.topic))
        print(s + ' : Stop to subscribe.')
    
    def start_subscribe(self):

        # Thread for waiting the first message
        self.ros_timer = Timer(5.0, self._start_ros)
        self.ros_timer.start()


@ray.remote(num_cpus=1) 
class _MqttToRosBridge_(object):
    def __init__(self, host:str, topic:str, color:str, max_length:int):
        if topic == '/imu/data':
            name        = 'imuData'
        else:
            name = topic.split('/')[-1]
            
        node_name   = ('_').join([name, 'bridge_node'])
        client_name = ('_').join([name, 'bridge_client'])

        rospy.init_node(node_name, anonymous=True)

        # Mqtt variables
        self.host   = host
        self.PORT   = 1883

        self.max_length     = max_length
        self.color          = color
        self.topic          = topic
        self.ROS_RATE       = rospy.Rate(1000)
        self.data           = self.create_dict()
        self.count          = 0
        self.len_data       = 0

        # Create mqtt client and build the mqtt connection
        self.client         = mqtt.Client(client_name)
        self.client.connect(self.host, self.PORT)

        # Create mqtt subscriber
        self.client.subscribe(self.topic)
        self.client.on_message      = self.on_messsage

        # Timer
        self.mqtt_timer     = Timer(10.0, self._start_mqtt)
        self.mqtt_timer.start()

    def create_dict(self) -> defaultdict:
        return defaultdict(self.create_dict)
    
    def _update_data(self, topic:str, message):
        self.data[topic] = message

    def get_data(self, topic:str) -> defaultdict:
        return self.data[topic]

    def stop_thread(self, flag:bool):
        self.client.disconnect()
        self.client.loop_stop()
        self.mqtt_timer.cancel()
    
    def _convert_mqtt_message(self, payload:str):
        """
        payload     : Mqtt payload
        """
        if any(self.message_type == x for x in ['Int32MultiArray', 'Float32MultiArray']):
            mqtt_message = json.loads(payload)
            return eval(self.message_type + '(data=np.array(mqtt_message))'), mqtt_message
        else:
            mqtt_message = eval(self.message_type + '(payload)')
            return mqtt_message, mqtt_message

    def process_message(self, topic:str, payload:str):
        """
        topic       : Topic name
        payload     : Mqtt payload
        """
        # Receive message for the first time
        if self.count == 0:
            s = topic + ' '*(self.max_length - len(topic))
            print(s + ' : Receiving message') 

            # Check type of message
            if payload.find('[') == 0 or payload.find('(') == 0 or payload.find(']') == 0 or payload.find(')') == 0:
                # Convert to list type
                mqtt_message    = json.loads(payload)
                if all(isinstance(var, int) for var in mqtt_message):
                    self.publisher      = rospy.Publisher(topic, Int32MultiArray, queue_size=1)
                    self.message_type   = 'Int32MultiArray'  
                    data                = Int32MultiArray(data=np.array(mqtt_message))
                elif all(isinstance(var, float) for var in mqtt_message):
                    self.publisher      = rospy.Publisher(topic, Float32MultiArray, queue_size=1)
                    self.message_type   = 'Float32MultiArray'
                    data                = Float32MultiArray(data=np.array(mqtt_message))
            else:
                mqtt_message    = payload
                if mqtt_message.replace('.','',1).isdigit():
                    if mqtt_message.isdigit():
                        mqtt_message        = int(mqtt_message)
                        self.publisher      = rospy.Publisher(topic, Int32, queue_size=1)
                        self.message_type   = 'int'
                        data                = int(mqtt_message)
                    else:
                        mqtt_message        = float(mqtt_message)
                        self.publisher      = rospy.Publisher(topic, Float32, queue_size=1)
                        self.message_type   = 'float'
                        data                = float(mqtt_message)
                else:
                    self.publisher      = rospy.Publisher(topic, String, queue_size=1)
                    self.message_type   = 'str'
                    data                = str(mqtt_message)
            self.publisher.publish(data)
            self.count = 1  
        else:
            [data, mqtt_message] = self._convert_mqtt_message(payload)
            
            # Publish ros message
            self.publisher.publish(data)
        
        return mqtt_message

    def on_messsage(self, client, userdata, message):
        topic       = message.topic
        payload_str = message.payload.decode()
        # print(topic, type(payload_str))
        mqtt_message = self.process_message(topic, payload_str)  
        self._update_data(topic, mqtt_message)
        self.len_data += 1


        self.mqtt_timer.cancel()
        # Start timer again
        self.mqtt_timer            = Timer(10.0, self._finished_from_mqtt)
        self.mqtt_timer.start()

    def _finished_from_mqtt(self):
        
        s = self.topic + ' '*(self.max_length - len(self.topic))
        print(s + f' : ({self.len_data} msgs) Transfer Finished from ROS and waiting for new message ...')
        self.count      = 0
        self.len_data   = 0

    def _start_mqtt(self):
        s = self.topic + ' '*(self.max_length - len(self.topic))
        print(s + ' : Waiting for message ...')

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
    
    def start_subscribe(self):
        self.client.loop_start()

class RosToMqttBridge(Thread):
    def __init__(self, host:str, topic:Dict[List[str], List[str]]=None, ray_state:bool=False):
        Thread.__init__(self)

        # Initialize ray
        if ray_state == False:
            self._initialize_ray()

        self.topic_dict     = topic
        self.color          = 'cyan'

        # MQTT variables
        self.host           = host
        self.PORT           = 1883
        self.QOS            = 0
        
        # Create the list of topic name and message library name
        self.ros_subscriber = dict()

        if self.topic_dict == None:
            print('\n Use auto-detect the ROS topics\' name and message types\n')
            tp_list = rospy.get_published_topics()
            self.topic_lst   = []
            self.msg_lib_lst = []
            for t, msg_lib in tp_list:
                if any(msg_lib.split('/')[0] == lib for lib in ['std_msgs','sensor_msgs']):
                    self.topic_lst.append(t)
                    self.msg_lib_lst.append(msg_lib)
        else:
            if search(list(self.topic_dict.keys()), 'topic') and search(list(self.topic_dict.keys()), 'message_type'):
                self.topic_lst   = self.topic_dict['topic']
                self.msg_lib_lst = self.topic_dict['message_type'] 
            else:
                print('Wrong json format !')

        print('______________________________________Start______________________________________\n')
        self.f = Figlet(font='slant')
        print(colored(self.f.renderText('ROS-MQTT bridge'), self.color))

        # Create ROS Subscriber
        self.max_length = max(len(s) for s in self.topic_lst)
        print('ROS topic(s) :')
        for (t, msg_lib) in zip(self.topic_lst ,self.msg_lib_lst):
            self._create_subscriber(t, msg_lib)
            s = t + ' '*(self.max_length - len(t))
            print(f'        {s} -> message : {msg_lib}')
        print('\n')
    
    def _initialize_ray(self, ):
        if ray.is_initialized():
            ray.shutdown()
        ray.init()
    
    def _get_num_cpus(self, msg_lib:str, num_topics:int) -> float:
        """
        msg_lib     : Message library of ROS.
                      For example :
                            - 'std_msgs/Int32'
        num_topics  : The total number of ROS topic.  
        """
        # Get the number of physical core in your computer
        self.cpu_count  = psutil.cpu_count(logical=False)

        if self.cpu_count/num_topics > 1:
            return 1
        else:
            [msg_pkg, msg_type] = msg_lib.split('/')
            if msg_pkg == 'std_msgs':
                if msg_type == 'Bool':
                    return 0.1
                elif any(msg_type == x for x in ['Int32', 'Float32', 'Int64', 'Float64']):
                    return 0.2
                elif any(msg_type == x for x in ['Int32MultiArray', 'Float32MultiArray', 'Int64MultiArray', 'Float64MultiArray']):
                    return 0.5
            elif msg_pkg == 'sensor_msgs':
                return 0.7
    
    def _get_message_type(self, message_lib:str) -> object:
        """
        message_lib : For example: 'std_msgs/...' or 'sensor_msgs/...'
        """
        msg_pkg, msg_type = message_lib.split('/')
        return getattr(import_module(msg_pkg+'.msg'), msg_type)
    
    def _create_subscriber(self, topic:str, msg_lib:str):
        self.num_cpus  = self._get_num_cpus(msg_lib, len(self.topic_lst))
        msg_pkg        = self._get_message_type(msg_lib)

        self.ros_subscriber[topic]  = _RosToMqttBridge_.options(num_cpus=self.num_cpus).remote(self.host, 
                                                                                          topic, 
                                                                                          msg_pkg, 
                                                                                          msg_lib, 
                                                                                          self.color, 
                                                                                          self.max_length)


    def get_data(self, topic:str):
        return ray.get(self.ros_subscriber[topic].get_data.remote(ray.put(topic)))

    def set_main(self, main:object):
        self.main = main
    
    def add_topic(self, topic:str, message_lib:str):
        if search(self.topic_lst, topic):
            pass
        else:
            # Add a new topic and message in list
            self.topic_lst.append(topic)
            self.msg_lib_lst.append(message_lib)    

            self.max_length = max(len(s) for s in self.topic_lst)

            # Create new ros subscriber
            self._create_subscriber(topic, message_lib)
            s = topic + ' '*(self.max_length - len(topic))
            print(colored('{Ros-to-Mqtt} ',self.color) + colored('Add topic -> ','red') + s + ' : message ' + message_lib)
            self.ros_subscriber[topic].start_subscribe.remote()
    
    def stop_actor(self):
        for t in self.topic_lst:
            self.ros_subscriber[t].stop_thread.remote(ray.put(True))

    def kill_ray(self):
        for t in self.topic_lst:
            ray.kill(self.ros_subscriber[t])
        print(colored('{_RosToMqttBridge_} Disconnected.', 'red'))

    def run(self):
        for t in self.topic_lst:
            self.ros_subscriber[t].start_subscribe.remote()

class MqttToRosBridge(Thread):
    def __init__(self, host:str, topic, ray_state:bool=False):
        Thread.__init__(self)
        # self.host       = '192.168.1.5'
        if ray_state == False:
            self._initialize_ray()

        self.host       = host
        self.PORT       = 1883
        self.color      = 'yellow'
        self.topic      = topic
        self.data       = dict()

        print('______________________________________Start______________________________________\n')
        self.f = Figlet(font='slant')
        print(colored(self.f.renderText('MQTT-ROS bridge'), self.color))

        # Create mqtt subscriber
        print('MQTT topic(s) :')
        self.ros_publisher  = dict()
        self.max_length     = max(len(s) for s in self.topic)

        for topic in self.topic:
            self._create_publisher(topic)
            s = topic + ' '*(self.max_length - len(topic))
            print(f'        {s}')

    def _initialize_ray(self):
        if ray.is_initialized():
            ray.shutdown()
        ray.init()
    
    def _get_num_cpus(self, num_topics:int) -> float:
        """
        num_topics  : The total number of topic
        """
        # Get the number of physical core in your computer
        self.cpu_count  = psutil.cpu_count()
        if num_topics < self.cpu_count:
            return 0.6
        else:
            num_cpus = self.cpu_count / num_topics
            if num_cpus >= 0.5:
                return 0.4
            elif num_cpus < 0.5 and num_cpus >= 0.25:
                return 0.2
    
    def _create_publisher(self, topic:str):
        self.num_cpus             = self._get_num_cpus(len(self.topic))
        # Create actor and override the default resource requirement
        self.ros_publisher[topic] = _MqttToRosBridge_.options(num_cpus=self.num_cpus).remote(self.host, topic, self.color, self.max_length)
    
    def add_topic(self, topic:str):
        if any(topic != t for t in self.topic):
            self.topic.append(topic)
            self.max_length = max(len(s) for s in self.topic)
            self._create_publisher(topic)
            s = topic + ' '*(self.max_length - len(topic))
            print(colored('{Mqtt-to-Ros} ',self.color) + colored('Add topic -> ','red') + s)

    def set_main(self, main:object):
        self.main = main
    
    def get_data(self, topic:str):
        return ray.get(self.ros_publisher[topic].get_data.remote(ray.put(topic)))
    
    def disconnect(self):
        for topic in self.topic:
            self.ros_publisher[topic].disconnect.remote()
        print(colored('{Mqtt-to-Ros} Client disconnected', 'red'))

    def run(self):
        for topic in self.topic:
            self.ros_publisher[topic].start_subscribe.remote()