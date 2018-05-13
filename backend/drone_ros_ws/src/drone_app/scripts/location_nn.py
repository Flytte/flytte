#!/usr/bin/env python
import rospy
import numpy as np
import base64, Image

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

from drone_app.srv import *
import vector

# --- Global variables initialization ---
# Filepath where images will be saved
filepath = "/home/aly/"
img_filepath = filepath
# Estimated pose
estimated_pose = vector.Vector(20, 20)
# ---------------------------------------

def random_position_gen():
    # --- Iteratively generate random position
    global estimated_pose
    x = estimated_pose.x + np.random.normal(0, 5, 1)[0]
    y = estimated_pose.y + np.random.normal(0, 5, 1)[0]
    estimated_pose = vector.Vector(sorted([0, x, 90])[1], sorted([0, y, 90])[1])

def create_message(position_arr):
    msg = PoseArray()
    # --- Pack all 4 points in an array
    for pose in position_arr:
        pose_msg = Pose()
        pose_msg.position.x = pose.x
        pose_msg.position.y = pose.y
        msg.poses.append(pose_msg)
    return msg

def estimate_pose():
    random_position_gen()
    global estimated_pose
    # Upper left corner
    position_arr = [estimated_pose]
    # Upper right corner
    position_arr.append(vector.Vector(position_arr[0].x + 10, position_arr[0].y))
    # Lower left corner
    position_arr.append(vector.Vector(position_arr[0].x, position_arr[0].y - 10))
    # Lower right corner
    position_arr.append(vector.Vector(position_arr[0].x - 10, position_arr[0].y - 10))
    return create_message(position_arr)

def parceRequest(req):
    global img_filepath

    img_filepath = filepath + "frame_location" + ".png"
    img = open(img_filepath, "wb")
    img.write(base64.b64decode(req.data.data.split(',')[1]))
    img.close()
    
    return ANNsResponse(estimate_pose())

if __name__ == '__main__':
    rospy.init_node('location_nn_server', anonymous=True)
    try:
        rospy.Service('location_nn', ANNs, parceRequest)
        rospy.loginfo("Location server ready...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")
        pass
