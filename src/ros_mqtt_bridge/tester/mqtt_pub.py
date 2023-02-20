import paho.mqtt.client as mqtt
from datetime import datetime
import sys
import time
import json
import rospy

if __name__ == '__main__':
    rospy.init_node('mqtt_publisher', anonymous=True)
    rate = rospy.Rate(1000)
    count = 0
    # Create MQTT client namely 'publisher'.
    client = mqtt.Client('publisher')
    if client.connect('localhost',1883, 60) != 0:
        print('Could not connect to mqtt broker')
        sys.exit(-1)

    # Defind topic name
    topic_msg = [('test1',[0.1,0.1,0.1,0.1,0.1,0.1]),
                ('test2', 100.6974562),
                ('test3',[100,100,100,100,100,100]),
                ('test4', -20),
                ('test5',[10.0,10.0,10.0,10.0,10.0,10.0]),
                ('test6','hello world')
                ]
    try:
        # Send message in loop.
        start_time = time.time()
        while True:
            if count == 0:
                print('Start to publish ...')
                count = 1
                prv_time = time.time()
            cur_time = time.time()
            if cur_time - start_time > 20:
                break
            else:
                for topic, msg in topic_msg:
                    ts = time.time()
                    # data = {'ts':ts, 'data':msg}
                    data = json.dumps(msg)
                    client.publish(topic, data)
            prv_time = cur_time
            rate.sleep()
        print('Finish !')

    except KeyboardInterrupt:
        # The client is disconnected if interrupt the keyboard.
        client.disconnect()