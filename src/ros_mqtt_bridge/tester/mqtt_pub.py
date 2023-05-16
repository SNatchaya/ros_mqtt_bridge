import paho.mqtt.client as mqtt
from datetime import datetime
import os
import time
import json

def on_connect(self, client, userdata, flags, rc):
    if rc != 0:
        print('Fail to connect MQTT broker')

if __name__ == '__main__':
    # Create MQTT client namely 'publisher'.

    client = mqtt.Client(client_id='mqtt_publisher')
    client.on_connect = on_connect
    client.connect('localhost', 1883)

    # Defind topic name
    topic = '/sub'
    while True:
        ms = 'Hello world ' + str(datetime.now())
        print(f'Topic: {topic} -> msg: {ms}')
        # Publish message
        client.publish(topic, ms, 0)
        time.sleep(1.5)
    # time.sleep(10)
    # try:
    #     while True:
    #         ms = 'Hello world ' + str(datetime.now())
    #         print(f'Topic: {topic} -> msg: {ms}')
    #         # Publish message
    #         client.publish(topic, ms, 0)
    #         time.sleep(1.5)

    # except KeyboardInterrupt:
    #     # The client is disconnected if interrupt the keyboard.
    #     client.disconnect()