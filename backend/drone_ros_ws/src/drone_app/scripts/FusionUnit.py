#!/usr/bin/env python
from DroneEKF import *
from DroneObserver import *

import rospy, tf

from drone_app_msgs.msg import *
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

# --- Global variables initialization ---
# Filter and observer
filterEK = None
observer = None
# Drone size
drone_width = 0
drone_height = 0
# Camera matrix
camera_matrix = np.zeros(4)
# Action vector
u = np.zeros(4)
# Observation vector odometry part
z_odom = np.zeros(4)
# Publisher
pub = None
# ---------------------------------------

def postFilteredPose(drone_name, pose):
    global drone_width, drone_height

    msg = DroneArray()

    # TODO create a for loop
    drone_msg = Drone()

    drone_msg.name = drone_name

    # --- Convert Pose to BBox
    bb_w = camera_fx * drone_width / pose[2]
    bb_h = camera_fy * drone_height / pose[2]

    drone_msg.box.t.linear.x = (camera_fx * pose[0] / pose[2] + camera_cx - bb_w / 2) * 100 / 640
    drone_msg.box.t.linear.y = (camera_fy * pose[1] / pose[2] + camera_cy - bb_h / 2) * 100 / 480

    drone_msg.box.w = bb_w * 100 / 640
    drone_msg.box.h = bb_h * 100 / 480

    drone_msg.box.t.angular.z = pose[3] * 180.0 / math.pi

    msg.drones.append(drone_msg)
    pub.publish(msg)

def useEKF(drone_name, z):
    global filterEK, observer, u, u_matrix

    # --- Predict ---
    filterEK.predict(u=u)

    # --- Update ---
    filterEK.update(z,
                    HJacobian = observer.jacobi_at,
                    Hx = observer.predict)

    # --- Send the updated pose ---
    postFilteredPose(drone_name, filterEK.x)

def saveRecognition(data):

    pose_estimation = np.zeros(5)

    pose_estimation[0] = data.drones[0].box.t.linear.x
    pose_estimation[1] = data.drones[0].box.t.linear.y
    pose_estimation[2] = data.drones[0].box.w
    pose_estimation[3] = data.drones[0].box.h
    pose_estimation[4] = data.drones[0].box.t.angular.z * math.pi / 180.0

    useEKF(data.drones[0].name, np.append(pose_estimation,z_odom))

def saveOdometry(data):
    global z_odom, z_odom_cov_matrix, u, u_cov_matrix

    # --- Convert to Client coordinates ---
    z_odom[0] = -1 * data.pose.pose.position.y
    z_odom[1] = data.pose.pose.position.x
    z_odom[2] = data.pose.pose.position.z

    # --- Convert from quaternion ---
    quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    z_odom[3] = euler[2]

    # --- Save the current velocity ---
    u[0] = data.twist.twist.linear.x
    u[1] = data.twist.twist.linear.y
    u[2] = data.twist.twist.linear.z
    u[3] = data.twist.twist.angular.z

if __name__ == '__main__':

    rospy.init_node('fusion_unit', anonymous=True)
    rospy.Subscriber('simulation_drones', DroneArray, saveRecognition)
    rospy.Subscriber('/bebop/odom', Odometry, saveOdometry)

    pub = rospy.Publisher('fixed_drones', DroneArray, queue_size = 10)

    filterEK = DroneEKF()

    std_vel =  rospy.get_param("/noise/std_nn_orientation")
    std_nn =   rospy.get_param("/noise/std_nn")
    std_odom = rospy.get_param("/noise/std_odom")

    if std_vel == None or std_nn == None or std_odom == None:
        raise RuntimeError("Noise information is not found")

    filterEK.R = np.diag([std_nn**2, std_nn**2, std_nn**2, std_nn**2, std_vel**2, std_odom**2, std_odom**2, std_odom**2, std_odom**2])

    camera_fx = rospy.get_param("/camera/fx")
    camera_fy = rospy.get_param("/camera/fy")
    camera_cx = rospy.get_param("/camera/cx")
    camera_cy = rospy.get_param("/camera/cy")

    if camera_fx == None or camera_fy == None or camera_cx == None or camera_cy == None:
        raise RuntimeError("Camera matrix are not found")

    camera_matrix = np.array([[camera_fx, camera_cx], [camera_fy, camera_cy]])

    # TODO: look for a drone depending on NNs output
    drone_width = rospy.get_param("/drones/parrot_bebop2/size/width")
    drone_height = rospy.get_param("/drones/parrot_bebop2/size/height")

    if drone_width == None or drone_height == None:
        raise RuntimeError("Drone parameters are not found")

    observer = DroneObserver(camera_matrix, drone_width, drone_height)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
