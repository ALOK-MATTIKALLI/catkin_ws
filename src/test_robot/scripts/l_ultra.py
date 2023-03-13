#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import Range

trig_l = 6
echo_l = 5
trig_c = 20
echo_c = 21
trig_r = 23
echo_r = 24

GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
GPIO.setup(trig_l, GPIO.OUT)
GPIO.setup(echo_l, GPIO.IN)
GPIO.setup(trig_c, GPIO.OUT)
GPIO.setup(echo_c, GPIO.IN)
GPIO.setup(trig_r, GPIO.OUT)
GPIO.setup(echo_r, GPIO.IN)

def ultra_distance(trig, echo):
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
    rospy.init_node("test_l_ultra_node")
    rospy.loginfo("test l_ultra node")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        # distance = ultra_distance()
        msg = Twist()
        msg.linear.x = round(ultra_distance(trig_l, echo_l),2)
        msg.linear.y = round(ultra_distance(trig_c, echo_c),2)
        msg.linear.z = round(ultra_distance(trig_r, echo_r),2)
        pub.publish(msg)
        rospy.loginfo(str(msg.linear.x) + "," + str(msg.linear.y) + "," + str(msg.linear.z))
        rate.sleep()
        # rospy.loginfo(distance)
        # if (distance > 10):
        #     msg = Twist()
        #     msg.linear.x = 1.0
        #     # msg.angular.z = -1.0
        #     pub.publish(msg)
        #     rospy.loginfo(msg.linear.x)
        #     rate.sleep()
        # else:
        #     msg = Twist()
        #     msg.linear.x = 0.0
        #     # msg.angular.z = 0.0
        #     pub.publish(msg)
        #     rospy.loginfo(msg.linear.x)
        #     rate.sleep()
