#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time


def talker(rate):
    pub = rospy.Publisher('chatter', String, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    ros_rate = rospy.Rate(int(rate)) # 10hz
    real_rate = list()
    
    start = 0
    
    while True:
        now = rospy.get_time()
        if start == 0:
            prv_time = rospy.get_time()
            start_time = rospy.get_time()
            start = 1
        else:
            if now - start_time > 20:
                break
            else:
                hello_str = "hello world : " + str(round((1/(now-prv_time)),4)) + ' Hz'
                real_rate.append(1/(now-prv_time))
                rospy.loginfo(hello_str)
                prv_time = now
                pub.publish(hello_str)
                ros_rate.sleep()
        # time.sleep(ts)
    print('Average of publish rate is : ',sum(real_rate[1:])/len(real_rate[1:]))
    print('Length is ',len(real_rate[1:]))

if __name__ == '__main__':
    rate = input('Enter rate : ')
    talker(rate)