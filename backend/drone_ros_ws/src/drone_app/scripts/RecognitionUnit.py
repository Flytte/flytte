#!/usr/bin/env python
import rospy
import base64, Image

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

from drone_app.srv import *

# --- Global variables initialization ---
# Current image count
image_count = 0
# Filepath where images will be saved
filepath = "/home/aly/"
image_filepath = filepath
# Path to the current frame
current_frame = String()
# Path to the cropped frame
cropped_frame = String()
# Boolean that shows whether there is a frame processed
frame_in_progress = False
# ---------------------------------------

def saveFrame(data):
    global frame_in_progress, image_count, current_frame

    if not frame_in_progress:
        image_count += 1
        current_frame = data

def crop_img(pose):
    global img_filepath, cropped_frame

    img_filepath = filepath + "frame_interface" + ".png"
    # --- Parse base64 to image
    img = open(img_filepath, "wb")
    img.write(base64.b64decode(current_frame.data.split(',')[1]))
    img.close()
    # --- Crop the image
    img = Image.open(img_filepath)
    # --- Recalculate from percentage back to pixels
    coeff = img.width / 100
    # --- Crop box (x, y, w+x, h+y) out
    img = img.crop((
        (int)(pose.poses[0].position.x * coeff),
        (int)(pose.poses[2].position.y * coeff),
        (int)(pose.poses[1].position.x * coeff),
        (int)(pose.poses[0].position.y * coeff)))
    img.save(img_filepath)

    cropped_frame = String(data = base64.b64encode(img.tobytes()))
    img.close()

if __name__ == '__main__':
    rospy.init_node('interface_nn', anonymous=True)
    rospy.wait_for_service('location_nn')
    try:
        # --- ROS Topics
        rospy.Subscriber('frames', String, saveFrame)
        pub = rospy.Publisher('estimated_pose', PoseArray, queue_size=10)
        location = rospy.ServiceProxy('location_nn', ANNs)
        orientation = rospy.ServiceProxy('orientation_nn', ANNs)
        # --- Main loop
        while not rospy.is_shutdown():
            if(image_count != 0):
                frame_in_progress = True
                # --- Receiving location estimation
                pose_response = location(current_frame)
                # --- Cropping the image
                crop_img(pose_response.pose)
                # --- Receiving orientation estimation
                orientation_response = orientation(cropped_frame)
                # --- Put together two messages
                pose_response.pose.poses[0].orientation = orientation_response.pose.poses[0].orientation
                # --- Publish
                pub.publish(pose_response.pose)
                frame_in_progress = False
    except rospy.ROSInterruptException:
        pass
