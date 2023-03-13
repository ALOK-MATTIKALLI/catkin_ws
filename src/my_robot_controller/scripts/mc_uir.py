#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

motorPins = [2,3,14,15]
GPIO.setmode(GPIO.BCM)
for i in motorPins:
    GPIO.setup(i, GPIO.OUT)

pwm_m1a = GPIO.PWM(motorPins[0],1000)
pwm_m1b = GPIO.PWM(motorPins[1],1000)
pwm_m2a = GPIO.PWM(motorPins[2],1000)
pwm_m2b = GPIO.PWM(motorPins[3],1000)

def robotDrive_callback(msg: Range):
    rospy.loginfo(f"Distance = {msg.range} cms")
    if msg.range > 20:
        print("Forward")
        motorControl(50,0,50,0)

    elif msg.range <= 20:
        print("STOP")
        # motorControl(GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW)
        motorControl(0,0,0,0)
        time.sleep(0.5)
        print("BACK")
        # motorControl(GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.HIGH)
        motorControl(0,50,0,50)
        time.sleep(1)
        print("LEFT")
        # motorControl(GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW)
        motorControl(0,50,50,0)
        time.sleep(0.5)

    

def motorControl(M1A,M1B,M2A,M2B):
    pwm_m1a.start(M1A)
    pwm_m1b.start(M1B)
    pwm_m2a.start(M2A)
    pwm_m2b.start(M2B)
    # GPIO.output(15,M2B)
    # GPIO.output(3,M1B)
    # GPIO.output(14,M2A)
    # GPIO.output(2,M1A)


if __name__ == "__main__":
    rospy.init_node("byor_controller_subscriber")
   
    # sub = rospy.Subscriber("/byor/sensor", Range, 
    rospy.loginfo("Node has been started!")
    rospy.spin()
    motorControl(0,0,0,0)
    GPIO.cleanup()
