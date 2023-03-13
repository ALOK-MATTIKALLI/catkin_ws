#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

m1a = 17
m1b = 27
m2a = 10
m2b = 9

l = 0
f = 0

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
def m_c(a2,b2,a1,b1):
    pwm_m1a.start(a1)
    pwm_m1b.start(b1)
    pwm_m2a.start(a2)
    pwm_m2b.start(b2)

def call_back(msg:Twist):
    global l, f
    # rospy.loginfo("(" + str(msg.linear.x) + "," + str(msg.linear.y) + "," + str(msg.linear.z) + ")")
    l = int(msg.linear.x)
    f = int(msg.linear.y)
    

if __name__ == '__main__':
    rospy.init_node("new_line_sub1")
    sub = rospy.Subscriber("/turtle1/cmd_vel", Twist, callback=call_back)
    rate = rospy.Rate(10)
    rospy.loginfo("new line motor node started")
    time.sleep(2.0)
    # less than 160 turn left
    f0 = int(f*0)
    f1 = int(f*0.1)
    f2 = int(f*0.2)
    f3 = int(f*0.3)
    f4 = int(f*0.4)
    f5 = int(f*0.5)
    f6 = int(f*0.6)
    f7 = int(f*0.7)
    f8 = int(f*0.8)
    f9 = int(f*0.9)
    f10 =int(f*1.0)
    
    while not rospy.is_shutdown():
        rospy.loginfo(l)
        if (l >f0 and l < f2):
            m_c(20,0,0,20)
        elif (l >f2 and l < f4+20): # left
            m_c(8,0,0,2)
        elif (l >f4+20 and l < f6+20): # forward
            m_c(8,0,8,0)
        elif (l >f6+20 and l < f8): # right
            m_c(0,2,8,0)
        elif (l >f8 and l < f10):
            m_c(0,20,20,0)
            
        rate.sleep()
        time.sleep(0.0001)
        
    # rospy.sleep()
rospy.spin()