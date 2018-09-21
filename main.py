#!/usr/bin/python

import rospy
import time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from itertools import groupby


emergency_stop = False
RANGE = 0.75

def speech():
    publisher_speech.publish("test")

def sonar_front(data):
    global emergency_stop
    if data.range < RANGE and emergency_stop is False:
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y= 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y= 0
        twist_msg.angular.z = 0

        print("EMERGENCY STOP FRONT", data.range)
        publisher_velocity.publish(twist_msg)
        speech()
        emergency_stop = True
        time.sleep(4)
        emergency_stop = False

def sonar_back(data):
    global emergency_stop
    if data.range < RANGE and emergency_stop is False:
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y= 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y= 0
        twist_msg.angular.z = 0

        print("EMERGENCY STOP BACK", data.range)
        publisher_velocity.publish(twist_msg)
        emergency_stop = True
        time.sleep(4)
        emergency_stop = False


def laser_detector(data):
    means = []
    ranges = [list(g) for k, g in groupby(data.ranges, lambda x: x != -1.0) if k]
    for r in ranges:
        tmp = np.array(r)
        means.append(np.mean(tmp[tmp < data.range_max]))
    
    print(means)

if __name__ == '__main__':
    rospy.init_node("collision_avoidance")
    publisher_speech = rospy.Publisher("/speech", String, queue_size=10)
    publisher_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # rospy.Subscriber("/pepper_robot/sonar/back", Range, sonar_back)
    # rospy.Subscriber("/pepper_robot/sonar/nt", Range, sonar_front)
    rospy.Subscriber("/pepper_robot/laser", LaserScan, laser_detector)
    rospy.spin()
