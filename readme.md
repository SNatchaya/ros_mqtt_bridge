# ROS-MQTT bridge

The ROS-MQTT bridge provides a feature that allows for bidirectional communication between ROS and MQTT.

---

## Getting Started

### Dependencies

- [x] ROS Neotic
- [x] Mosquitto MQTT
- [x] Python version > 3.7
    - numpy==1.23.4
    - wheel==0.34.2
    - paho_mqtt==1.5.0
    - pyfiglet==0.8.post1
    - rospy==1.15.14
    - termcolor==2.1.0
    - pyyaml==5.3.1
    - rospkg==1.4.0
    - netifaces==0.10.4

### Installation

1. Clone the whole file from [GitLab](https://gitlab.com/BRAIN_Lab/biorobotics-private/exobic) to your local computer.

```sh
git clone xxx
```

2. Install the needed library from the requirements.txt file.

```sh
cd RosMqttBridge
```
```sh
pip install -r requirements.txt 
#or
pip3 install -r requirements.txt
```

---

## Demo

<!-- > Note: launch the `test_script.py` before launch the `bridge.py`, due to ROS-MQTT bridge can get the topic and message type from ros automatically. -->

### Prelaunch

> If an error appears in the MQTT broker's terminal like `address already in use error in mosquitto`, it means that your broker is still running **(this error usually appears when you use the broker for the first time after powering on or restarting your computer)**. To solve the error, please open a new terminal and enter the following command.

```sh
sudo systemctl stop mosquitto
```

After entering this command, you can move on to the next step and run the demo.

### Step-by-Step

1. Run `demo.py` to start the demo.

    ```sh
    python3 demo.py
    ```

    After running the `demo.py`, the roscore's terminal (or not), MQTT broker's terminal (or not), a ROS publisher's terminal, a MQTT publisher's terminal, a ROS subsciber's terminal and a MQTT subscriber's terminal are launched.   

    Each terminal means ...

    | Window | Description |
    | ------ | ----------- |
    | roscore           | ROS server |
    | MQTT broker       | MQTT broker (mosquitto) for handling the whole message. |
    | ROS publisher     | Publish message via ROS using `rosbag_test.bag`. |
    | MQTT publisher    | Publish message via MQTT using `mqtt_pub.py`. |
    | ROS subscriber    | Subscribe topic from MQTT publisher <`command: rostopic echo /topic_name`>. |
    | MQTT subscribe    | Subscribe topic from ROS publisher <`command: mosquitto_sub -h host_IP -t topic_name`>. |

    > Note: This demo runs the ROS publisher before starting the bridge, which bridge automatically gets topics and message types. 

2. To stop the whole process, follow the stop process in the main terminal (the terminal where `demo.py` is running).
    - Press **s** to stop the ROS-MQTT bridge process.
    - Press **Ctrl+C** to stop and exit the process in the other terminal. In addition, some terminals will automatically close by themselves when the process ends.