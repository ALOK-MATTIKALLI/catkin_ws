#! /usr/bin/env python3
import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Range
#from std_msgs.msg import Float32

if __name__ == "__main__":
    rospy.init_node("ultrasound_node")
    rospy.loginfo("Ultrasound node has been started!")
    trig = 20
    echo = 21
    GPIO.setmode(GPIO.BCM)
    GPIO.cleanup()
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo,GPIO.IN)

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

    pub = rospy.Publisher("/byor/us_sensor",Range,queue_size=None)
    rate = rospy.Rate(8)

    while not rospy.is_shutdown():
        us_msg = Range()
        us_msg.range = ultra_distance()
    
        if us_msg.range > 20:
            pub.publish(us_msg)
            rospy.loginfo("Distance = %0.4f cms" %(us_msg.range))
        
        elif us_msg.range <= 20:
            pub.publish(us_msg)
            rospy.loginfo("Distance = %0.4f cms" %(us_msg.range))
            time.sleep(1)
        rate.sleep()