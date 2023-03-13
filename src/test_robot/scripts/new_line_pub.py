#!/usr/bin/env python3

import rospy # Python library for ROS
import cv2 # OpenCV library
from geometry_msgs.msg import Twist

def publish_message():
    rospy.init_node("camera command")
    rospy.loginfo("camera command node started")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    cap = cv2.VideoCapture(0)
    # cap.set(3, 320)
    # cap.set(4, 240)
    intc = 30

    # While ROS is still running.
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret == True:
            l=0
            r=0
            d = 0
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # cv2.imshow('frame', gray)
            dim = gray.shape
            arr = gray[int(80*dim[0]/100)]
            rag = int(len(arr)*15/100)
            for j in range (0, len(arr)):
                if (arr[j] < intc):
                    l = j
                    break
            # print("left = " + str(l))
            for j in range (j, len(arr)):
                if (arr[j] > intc):
                    r = j-1
                    break
            # print('right = ' + str(r))
            l_cent = int((l+r)/2)
            # print("line center = {}".format(l_cent))
            f_cent = int(len(arr))
            msg = Twist()
            # if l ==0 and r>635 and l_cent == int(r/2):
            #    if d== -1:
            #       msg.linear.x = 500
            #       msg.linear.y = f_cent
            #    elif d==1:
            #       msg.linear.x = 100
            #       msg.linear.y = f_cent
            # else:
            #    msg.linear.x = l_cent
            #    msg.linear.y = f_cent
            msg.linear.x = l_cent
            msg.linear.y = f_cent
            
            rospy.loginfo(l_cent)

            if (l_cent < f_cent):d=-1
            elif(l_cent >f_cent): d= 1
            else: d = 0
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass