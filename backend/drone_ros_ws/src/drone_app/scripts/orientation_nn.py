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
estimated_orientation = vector.Vector(0, 0, 0)
# ---------------------------------------

def random_orientation_gen():
    # --- Iteratively generate random orientation
    global estimated_orientation
    x = estimated_orientation.x + np.random.normal(85, 95, 1)[0]
    y = estimated_orientation.y + np.random.normal(0, 5, 1)[0]
    z = estimated_orientation.z + np.random.normal(0, 5, 1)[0]
    estimated_orientation = vector.Vector(sorted([0, x, 90])[1], sorted([0, y, 90])[1], sorted([0, z, 90])[1])

def create_message():
    # --- Pack it in a message
    msg = PoseArray()

    pose_msg = Pose()
    pose_msg.orientation.x = estimated_orientation.x
    pose_msg.orientation.y = estimated_orientation.y
    pose_msg.orientation.z = estimated_orientation.z
    msg.poses.append(pose_msg)

    return msg

def estimate_pose():
    random_orientation_gen()

    return create_message()

def parceRequest(req):
    global img_filepath

    img_filepath = filepath + "frame_orientation" + ".png"
    img = open(img_filepath, "wb")
    img.write(base64.b64decode(req.data.data))
    img.close()

    return ANNsResponse(estimate_pose())

if __name__ == '__main__':
    rospy.init_node('orientation_nn_server', anonymous=True)
    try:
        rospy.Service('orientation_nn', ANNs, parceRequest)
        rospy.loginfo("Orientation server ready...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException")
        pass
