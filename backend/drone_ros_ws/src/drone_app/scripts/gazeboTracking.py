#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from drone_app_msgs.msg import BBox, Drone, DroneArray
from rospy.numpy_msg import numpy_msg

# ---------------------------------------
# This is an implementation of a simple CV
# algorithm that can be used for testing
# --- Global variables initialization ---
pub = None
# ---------------------------------------

def processFrame(image_message):
    # --- Convert from ROS to OpenCV
    frame = CvBridge().imgmsg_to_cv2(image_message)

    # --- Threshold the image and find a mask
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, (0, 0, 0, 0), (180, 255, 30, 0))
    mask = cv2.dilate(mask, None, iterations=1)

    # --- Find contours in the mask and initialize the current
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    c = max(cnts, key=cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)

    # --- Pack in the message
    msg = DroneArray()
    drone = Drone()

    drone.id = -1
    drone.name = 'parrot_bebop2'
    drone.box.t.linear.x = x * 100 / 640
    drone.box.t.linear.y = y * 100 / 480
    drone.box.w = w * 100 / 640
    drone.box.h = h * 100 / 480

    msg.drones.append(drone)
    pub.publish(msg)

if __name__ == '__main__' :
    # --- Topics
    rospy.init_node('gazeboTracking', anonymous=True)
    rospy.Subscriber('camera_img', Image, processFrame)
    pub = rospy.Publisher('fixed_drones', DroneArray, queue_size=10)
    
    rospy.spin()
