#!/usr/bin/env python
import roslib; roslib.load_manifest('kinect_pitch')
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from dynamixel_msgs.msg import JointState

import numpy as np

kinect_yaw = 0.0
desired_head_angle = np.pi*30/180
kinect_pitch = 0.0
update_freq = 0.1

def IMUcallback(data):
    # pulish here
    [q0,q1,q2,q3] = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    phi = np.arctan2( 2*(q0*q1 + q2*q3) , 1 - 2*(q1**2+q2**2)) # rotation around X-axis
    theta = np.arcsin( 2* (q0*q2 - q3*q1) )                    # rotation around Y-axis
    psi = np.arctan2( 2*(q0*q3 + q1*q2) , 1 - 2*(q2**2+q3**2)) # rotation around Z-axis
    robot_pitch = theta
    robot_roll  =  psi

    if robot_roll < 0:
        robot_roll += np.pi
    else:
        robot_roll -= np.pi

    global kinect_yaw, kinect_pitch
    # rospy.loginfo(rospy.get_name() + ": I heard %f" % robot_roll)
    kinect_pitch = - robot_pitch * np.cos(kinect_yaw) + robot_roll * np.sin(kinect_yaw)

    # if pub is not None:
    #     global desired_head_angle
    #     pub.publish(kinect_pitch - desired_head_angle)
        

def kinectyawcallback(data):
    global kinect_yaw
    kinect_yaw = data.current_pos
   
def Initialize():
    rospy.init_node('kinect_pitch', anonymous=True)

    rospy.Subscriber("/imu_data", Imu, IMUcallback)
    rospy.Subscriber("/sh_yaw_controller/state", JointState , kinectyawcallback)

    pub = rospy.Publisher('/sh_pitch_controller/command', Float64)
    while not rospy.is_shutdown():
        global desired_head_angle, kinect_pitch, update_freq
        pub.publish(kinect_pitch - desired_head_angle)
        rospy.sleep(update_freq)

    rospy.spin()

if __name__ == '__main__':
    try:
        Initialize()
    except rospy.ROSInterruptException:
        pass
