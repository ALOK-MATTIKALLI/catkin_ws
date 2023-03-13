#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import RPi.GPIO as GPIO
import time

m1a = 17
m1b = 27
m2a = 10
m2b = 9

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


def callback(data):
    l = 0
    r = 0
    intc = 50

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # Display image
    # cv2.imshow("camera", current_frame)
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('frame', gray)
    dim = gray.shape
    arr = gray[int(80*dim[0]/100)]
    rag = int(len(arr)*6/100)
    for j in range (0, len(arr)):
        if (arr[j] < intc):
            l = j
            break
    for j in range (j, len(arr)):
        if (arr[j] > intc):
            r = j-1
            break
    l_cent = int((l+r)/2)
    f_cent = int(len(arr)/2)
    if (l_cent < (f_cent-rag)):
        print("turn right")
        m_c(0,0,20,0)
        # time.sleep(0.5)
    elif (l_cent > (f_cent + rag)):
        print("turn left")
        m_c(20,0,0,0)
        # time.sleep(0.5)
    elif ( (l_cent < (f_cent + rag)) and (l_cent > (f_cent - rag)) ):
        print("forward")
        m_c(20,0,20,0)
        # time.sleep(0.5)
    else:
        print('stop')
        m_c(0,0,0,0)
        # time.sleep(0.5)

    # cv2.waitKey(1)


def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name. 
    rospy.init_node('line_video_sub_py')

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('video_frames', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()

  
if __name__ == '__main__':
  receive_message()