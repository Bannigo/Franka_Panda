#!/usr/bin/env python

import cv2
import rospy
import sympy as sp
import numpy as np
import tf
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image
from numpy import *
from nav_msgs.msg import Odometry
def Odometry_callback(msg):
    print(msg)

if __name__ == '__main__':
    rospy.init_node('laser_scan')
    

    Odometry_sub = rospy.Subscriber('/odom', Odometry, Odometry_callback)
    rospy.spin()
"""

def make_rotation_matrix(rotation_angle):
    
    roll = rotation_angle[0]
    pitch = rotation_angle[1]
    yaw = rotation_angle[2]

    yawMatrix = np.matrix([
    [math.cos(yaw), -math.sin(yaw), 0],
    [math.sin(yaw), math.cos(yaw), 0],
    [0, 0, 1]
    ])

    pitchMatrix = np.matrix([
    [math.cos(pitch), 0, math.sin(pitch)],
    [0, 1, 0],
    [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rollMatrix = np.matrix([
    [1, 0, 0],
    [0, math.cos(roll), -math.sin(roll)],
    [0, math.sin(roll), math.cos(roll)]
    ])

    R = yawMatrix * pitchMatrix * rollMatrix
    return R


quat = [0.892, -0.131, -0.005, -0.433]
euler = tf.transformations.euler_from_quaternion(quat)
matrix = quaternion_matrix(quat)
print(matrix)


theta1, theta2, theta3, theta4, theta5, theta6, theta7 = sp.symbols("theta1 theta2 theta3 theta4 theta5 theta6 theta7")


expand =   [0.107*sp.sin(theta2)*sp.sin(theta3)*sp.sin(theta5)*sp.sin(theta6), + 
            0.088*sp.sin(theta2)*sp.sin(theta3)*sp.sin(theta5)*sp.cos(theta6), + 
            0.088*sp.sin(theta2)*sp.sin(theta4)*sp.sin(theta6)*sp.cos(theta3), - 
            0.107*sp.sin(theta2)*sp.sin(theta4)*sp.cos(theta3)*sp.cos(theta6), + 
            0.384*sp.sin(theta2)*sp.sin(theta4)*sp.cos(theta3), - 
            0.107*sp.sin(theta2)*sp.sin(theta6)*sp.cos(theta3)*sp.cos(theta4)*sp.cos(theta5), - 
            0.088*sp.sin(theta2)*sp.cos(theta3)*sp.cos(theta4)*sp.cos(theta5)*sp.cos(theta6), + 
            0.0825*sp.sin(theta2)*sp.cos(theta3)*sp.cos(theta4), - 
            0.0825*sp.sin(theta2)*sp.cos(theta3), + 
            0.107*sp.sin(theta4)*sp.sin(theta6)*sp.cos(theta2)*sp.cos(theta5), + 
            0.088*sp.sin(theta4)*sp.cos(theta2)*sp.cos(theta5)*sp.cos(theta6), - 
            0.0825*sp.sin(theta4)*sp.cos(theta2), + 
            0.088*sp.sin(theta6)*sp.cos(theta2)*sp.cos(theta4), - 
            0.107*sp.cos(theta2)*sp.cos(theta4)*sp.cos(theta6), + 0.384*sp.cos(theta2)*sp.cos(theta4), + 0.316*sp.cos(theta2)]

theta = [theta1,theta2,theta3,theta4,theta5,theta6,theta7]

for i in theta:
    print("Doing " + str(i) + " \n ")
    for j in expand:
        print(j.diff(i))

theta1 = 0.2124168449737
theta2 = -0.287595794476
theta3 = 0.21242332855316
theta4 = -2.28759613652233
theta5 = -0.212422726668573
theta6 = 2.21239931922184
theta7 = 0.212397392589472

"""

