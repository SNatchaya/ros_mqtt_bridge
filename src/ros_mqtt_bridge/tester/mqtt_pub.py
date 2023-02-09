import paho.mqtt.client as mqtt
from datetime import datetime
import sys
import time

# Create MQTT client namely 'publisher'.
client = mqtt.Client('publisher')
if client.connect('localhost',1883, 60) != 0:
    print('Could not connect to mqtt broker')
    sys.exit(-1)

# Defind topic name
topic = '/sub'
try:
    # Send message in loop.
    while True:
        ms = 'Hello world ' + str(datetime.now())
        print(f'Topic: {topic} -> msg: {ms}')
        # Publish message
        client.publish(topic, ms, 0)
        time.sleep(1.5)

except KeyboardInterrupt:
    # The client is disconnected if interrupt the keyboard.
    client.disconnect()