#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

trig = 23
echo = 24
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

def ultra_distance():
    GPIO.output(trig,False)
    time.sleep(0.00002)
    GPIO.output(trig,True)
    time.sleep(0.00005)
    GPIO.output(trig,False)
    
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()

    while GPIO.input(echo) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300)/2
    return distance

if __name__ == '__main__':
    rospy.init_node("test_r_ultra_node")
    rospy.loginfo("test r_ultra node")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        distance = ultra_distance()
        # rospy.loginfo(distance)
        if (distance > 10):
            msg = Twist()
            msg.angular.z = 1.0
            pub.publish(msg)
            rospy.loginfo(msg.linear.x)
            rate.sleep()
        else:
            msg = Twist()
            msg.angular.z = 0.0
            pub.publish(msg)
            rospy.loginfo(msg.linear.x)
            rate.sleep()
