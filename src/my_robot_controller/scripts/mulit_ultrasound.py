#! /usr/bin/env python3
import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range

if __name__ == "__main__":
    rospy.init_node("multisensor_node")
    rospy.loginfo("Ultrasound node has been started!")
    us_controlPins = [6,5,20,21,23,24]
    GPIO.setmode(GPIO.BCM)

    for i in range(0,6,2):
        GPIO.setup(us_controlPins[i], GPIO.OUT)

    for i in range(1,6,2):
        GPIO.setup(us_controlPins[i], GPIO.IN)

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
    

    pub = rospy.Publisher("/byor/multi_us",Range,queue_size=None)
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        us1_msg = Range()
        us1_msg.range = ultra_distance(us_controlPins[0], us_controlPins[1])
    
        us2_msg = Range()
        us2_msg.range = ultra_distance(us_controlPins[2], us_controlPins[3])

        us3_msg = Range()
        us3_msg.range = ultra_distance(us_controlPins[4], us_controlPins[5])
        pub.publish(us1_msg)
        pub.publish(us2_msg)
        pub.publish(us3_msg)
        rospy.loginfo("leftDist = %0.4f cms, middleDist = %0.4f cms, rightDist = %0.4f cms" %(us1_msg.range, us2_msg.range, us3_msg.range))
        # if us_msg.range > 20:
        #     pub.publish(us_msg)
        #     rospy.loginfo("Distance = %0.4f cms" %(us_msg.range))
        
        # elif us_msg.range <= 20:
        #     pub.publish(us_msg)
        #     rospy.loginfo("Distance = %0.4f cms" %(us_msg.range))
        #     time.sleep(1)
        rate.sleep()

    GPIO.cleanup()
        
    