#!/usr/bin/env python

import rospy
import math     
import numpy as np
import sympy as sp
import tf
from franka_msgs.srv import jacobian_service
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


global states_joint

def states_callback(msg):
    global states_joint
    states_joint = msg.position

def handle_pose_to_cartesian(thetas):
   
    # DH parameters of Franka_Panda, thetas are the incoming values of theta from jointstates topic
    m01 = calculate_matrix(0 ,0 ,0.333, thetas[0]) 
    m12 = calculate_matrix(0 ,-math.radians(90), 0, thetas[1])
    m23 = calculate_matrix(0 ,math.radians(90), 0.316, thetas[2])
    m34 = calculate_matrix(0.0825 ,math.radians(90), 0, thetas[3] )
    m45 = calculate_matrix(-0.0825 ,-math.radians(90), 0.384, thetas[4])
    m56 = calculate_matrix(0 ,math.radians(90), 0, thetas[5])
    m67 = calculate_matrix(0.088 ,math.radians(90), 0,  thetas[6])
    m78 = calculate_matrix(0, 0, 0.107, 0)

    # Multiplication of transformation matrix
    m02 = np.dot(m01,m12)
    m03 = np.dot(m02,m23)
    m04 = np.dot(m03,m34)
    m05 = np.dot(m04,m45)
    m06 = np.dot(m05,m56)
    m07 = np.dot(m06,m67)
    m08 = np.dot(m07,m78) #The transformation matric from panda_link0 to panda_link8

    return m08 # Print transformation matrices 

def calculate_matrix(a, alpha, d, theta): 
    matrix =    [[math.cos(theta),             -math.sin(theta),         0,           a ],
                [math.sin(theta)*math.cos(alpha),  math.cos(theta)* math.cos(alpha),  -math.sin(alpha), -math.sin(alpha)*d],
                [math.sin(theta)*math.sin(alpha),  math.cos(theta)* math.sin(alpha) ,  math.cos(alpha),  math.cos(alpha)*d],
                [0  ,                    0    ,                    0     ,      1      ]        ]
    return matrix

if __name__ == '__main__':

    rospy.init_node('incremental')
    sub = rospy.Subscriber("/joint_states", JointState, states_callback)
    rospy.sleep(2)
    final_pose = [[ 0.48262289 , 0.7117219 ,  0.51041853 , 0.59919708],
                  [ 0.66032565 ,-0.67854436 , 0.32178811 , 0.42222169],
                  [ 0.57536526 , 0.18174014 ,-0.79744927 , 0.51421959],
                  [ 0.     ,     0.     ,     0.      ,    1.        ]]


    init_pose = [[ 0.99432129,  0.0803485 ,  0.06978027,  0.41552049],
                 [ 0.08535554 ,-0.99374488 ,-0.07201073 , 0.04879733],
                  [ 0.06355783 , 0.07755793, -0.99495988 , 0.46771329],
                  [ 0.      ,    0.   ,       0.     ,     1.        ]]

                
    move_pub = rospy.Publisher("/joint_position_example_controller_sim/joint_command", Float64MultiArray, queue_size = 1000)

    #current_joint = [ 2.46968631, 0.74728895, -1.94441554 ,-1.79468912 , 0.47014124  ,1.73323531, 0.1521456 ]
    global states_joint
    current_joint = states_joint
    init_pose = handle_pose_to_cartesian(current_joint)
    print(init_pose)
    result_matrix = np.subtract(final_pose, init_pose)
    
    Delta = np.amax(abs(result_matrix))
    Delta_enter = Delta
    print(Delta)
    rospy.wait_for_service("/compute_another_jacobian")
    compute_jacobian = rospy.ServiceProxy("/compute_another_jacobian", jacobian_service)
    i = 0
    while Delta > 0.01:
        i = i+1
        jacobian_vectors = compute_jacobian(current_joint)
        Jacobian = [jacobian_vectors.a1, jacobian_vectors.a2, jacobian_vectors.a3, jacobian_vectors.a4, jacobian_vectors.a5, jacobian_vectors.a6, jacobian_vectors.a7,jacobian_vectors.a8,jacobian_vectors.a9,jacobian_vectors.a10,jacobian_vectors.a11,jacobian_vectors.a12]
        result_matrix = np.subtract(final_pose, init_pose)
        result_matrix = result_matrix[0:3 , 0:4]
        
        A_vector = np.array([[ result_matrix[0][0], result_matrix[1][0], result_matrix[2][0], result_matrix[0][1], result_matrix[1][1], result_matrix[2][1],
                     result_matrix[0][2], result_matrix[1][2], result_matrix[2][2], result_matrix[0][3], result_matrix[1][3], result_matrix[2][3] ]])
        print(A_vector.T)
        psd = np.linalg.pinv(Jacobian)
        theta_diff = np.dot(psd, A_vector.T)
        current_joint = np.add(current_joint, theta_diff.T[0])
        current_joint = np.unwrap(current_joint)
        
        if not -2.8973 <= current_joint[0] <= 2.8973 :
            current_joint[0] = np.random.uniform(2.8973, -2.8973)
        
        if not -1.7628 <= current_joint[1] <= 1.7628 :
            current_joint[0] = np.random.uniform(1.7628, -1.7628)

        if not -2.8973 <= current_joint[2] <= 2.8973 :
            current_joint[0] = np.random.uniform(2.8973, -2.8973)

        if not -3.0718 <= current_joint[3] <= -0.0698 :
            current_joint[3] = np.random.uniform(-0.0698, -3.0718)
        
        if not -2.8973 <= current_joint[4] <= 2.8973 :
            current_joint[0] = np.random.uniform(2.8973, -2.8973)
        
        if not -0.0175 <= current_joint[5] <= 3.7525:
            current_joint[0] = np.random.uniform(3.7525, -0.0175)

        if not -2.8973 <= current_joint[6] <= 2.8973 :
            current_joint[0] = np.random.uniform(2.8973, -2.8973)
        
        init_pose = handle_pose_to_cartesian(current_joint)
        result_matrix = np.subtract(final_pose, init_pose)
        Delta = np.amax(abs(result_matrix))
        print(i)

    print(current_joint)
    print(handle_pose_to_cartesian(current_joint))
    msg = Float64MultiArray()
    
    new_joint = np.unwrap(current_joint)
    print(new_joint)
    print(handle_pose_to_cartesian(new_joint))
    
    print(Delta_enter)
    while not rospy.is_shutdown():
        msg.data = [new_joint[0], new_joint[1], new_joint[2], new_joint[3], new_joint[4], new_joint[5], new_joint[6]]
        move_pub.publish(msg)
    
    #rospy.sleep(2)
    
    print("Done")
    """0.47970336661573487, -0.9797034806673945, -0.4797032516873818, -2.9797032736683056, 0.47970326035297006, 1.5202967131543055, -0.4797033926199985]

    [-

    [0.4761070991087859, -0.023892698094547526, 0.4761071080542729, -2.023893344297366, -0.47610712307286107, 2.4761068890712465, 0.47610691789749104]

    """