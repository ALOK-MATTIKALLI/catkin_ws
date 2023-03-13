#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("The test node started!")
    motorPins = [2,3,14,15]

    GPIO.setmode(GPIO.BCM)
    for i in motorPins:
        GPIO.setup(i, GPIO.OUT)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        GPIO.output(15,GPIO.LOW)
        GPIO.output(3,GPIO.LOW)
        GPIO.output(14,GPIO.HIGH)
        GPIO.output(2,GPIO.HIGH)
        rospy.loginfo("Running forward")
        rate.sleep()
        GPIO.output(15,GPIO.LOW)
        GPIO.output(3,GPIO.LOW)
        GPIO.output(14,GPIO.LOW)
        GPIO.output(2,GPIO.LOW)
        rospy.loginfo("Stopped")
        rate.sleep()



