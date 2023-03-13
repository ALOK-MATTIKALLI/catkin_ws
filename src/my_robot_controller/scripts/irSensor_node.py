#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int16
import RPi.GPIO as GPIO
import time

if __name__ == "__main__":
    rospy.init_node("IRSensor_node")
    rospy.loginfo("Reading IR sensor node started!")
    leftIR = 2
    rightIR = 3
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(leftIR, GPIO.IN)
    GPIO.setup(rightIR,GPIO.IN)

    pub = rospy.Publisher("/byor/ir_sensor",Int16,queue_size=None)
    rate = rospy.Rate(8)

    while not rospy.is_shutdown():
        
        if (GPIO.input(leftIR) == GPIO.HIGH) and (GPIO.input(rightIR) == GPIO.LOW):
            rospy.loginfo("LEFT")
            ir_msg = 1
            pub.publish(ir_msg)

        elif (GPIO.input(leftIR) == GPIO.LOW) and (GPIO.input(rightIR) == GPIO.HIGH):
            rospy.loginfo("RIGHT")
            ir_msg = 2
            pub.publish(ir_msg)

        elif (GPIO.input(leftIR) == GPIO.HIGH) and (GPIO.input(rightIR) == GPIO.HIGH):
            rospy.loginfo("BOTH")
            ir_msg = 3
            pub.publish(ir_msg)

        else:
            rospy.loginfo("NONE")
            ir_msg = 4
            pub.publish(ir_msg)

        rate.sleep()

GPIO.cleanup()
