#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

m1a = 17
m1b = 27
m2a = 10
m2b = 9

l_l = 0.0
l_f = 0.0
l_r = 0.0

GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
GPIO.setup(m1a, GPIO.OUT)
GPIO.setup(m1b, GPIO.OUT)
GPIO.setup(m2a, GPIO.OUT)
GPIO.setup(m2b, GPIO.OUT)
pwm_m1a = GPIO.PWM(m1a, 1000)
pwm_m1b = GPIO.PWM(m1b, 1000)
pwm_m2a = GPIO.PWM(m2a, 1000)
pwm_m2b = GPIO.PWM(m2b, 1000)

GPIO.setwarnings(False)            #do not show any warnings
def m_c(a1,b1,a2,b2):
    pwm_m1a.start(a1)
    pwm_m1b.start(b1)
    pwm_m2a.start(a2)
    pwm_m2b.start(b2)

def call_back(msg:Twist):
    global l_l, l_f, l_r
    rospy.loginfo("(" + str(msg.linear.x) + "," + str(msg.linear.y) + "," + str(msg.linear.z) + ")")
    l_l = msg.linear.x
    l_f = msg.linear.y
    l_r = msg.linear.z


if __name__ == '__main__':
    rospy.init_node("ultra_motor")
    sub = rospy.Subscriber("/turtle1/cmd_vel", Twist, callback=call_back)
    rate = rospy.Rate(4)
    rospy.loginfo("ultra_motor node started")

    while not rospy.is_shutdown():
        if ((l_l < 20.0) & (l_r < 20.0) & (l_r < 20.0)):
            m_c(0,0,0,0)
            time.sleep(0.5)
            m_c(0,40,0,40)
            time.sleep(1.0)
            m_c(40,0,0,0)
            time.sleep(1.0)
        elif (l_l < 20.0):
            m_c(40,0,0,0)
            time.sleep(1.0)
        elif (l_r < 20.0):
            m_c(0,0,40,0)
            # time.sleep(1.0)
        elif (l_f < 20.0):
            m_c(0,40,0,40)
            # time.sleep(1.0)
        else:
            m_c(40,0,40,0)
            # time.sleep(1.0)

            

        rospy.loginfo(str(l_l) + "," + str(l_f) + "," + str(l_r))
        # time.sleep(0.5)
        rate.sleep()
        
    # rate.sleep()
    # rospy.sleep()
    rospy.spin()