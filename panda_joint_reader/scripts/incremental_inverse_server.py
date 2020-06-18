#!/usr/bin/env python

import rospy
import math     
import numpy as np
import sympy as sp
import tf
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import derivative_matrix
from franka_msgs.srv import jacobian_service
from franka_msgs.srv import inverse_kinematics
from franka_msgs.srv import cartesianfrompose
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

global current_joint, goal_pose , current_pose

def states_callback(msg):
    global current_joint, goal_pose , current_pose
    current_joint = msg.position
    try:
        current_pose = pose_to_cartesian(msg.position).pose
        
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
        pass



def compute_least_square(Jacobian, vector):

    psd = np.linalg.pinv(Jacobian)

    x  = np.dot(psd,vector.T)
    array = []
    for value in x.T:
        new_value = value % math.pi
        array.append(new_value)


    return array    
    
def extract_elements(matrix):
    print(matrix)
    da11 = matrix[0]
    da12 = matrix[1]
    da13 = matrix[2]
    da21 = matrix[3]
    da22 = matrix[4]
    da23 = matrix[5]
    da31 = matrix[6]
    da32 = matrix[7]
    da33 = matrix[8]
    da14 = matrix[9]
    da24 = matrix[10]
    da34 = matrix[11]
    b = np.array([[da11,da21,da31,da12,da22,da32,da13,da23,da33,da14,da24,da34]])
    #print(b)
    return b


def differential_error(goal, current):
    print(current)
    print(goal)
    x1 = goal.position.x
    y1 = goal.position.y
    z1 = goal.position.z
    x2 = current.position.x
    y2 = current.position.y
    z2 = current.position.z
    differential_error =  math.sqrt(math.pow((x1 - x2),2) + math.pow((y1 - y2),2) + math.pow((z1 - z2),2))
    #print(differential_error)
    return differential_error

def difference_matrix(goal, current):
    global current_joint, goal_pose , current_pose
    current_quaternion = [current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w]
    goal_quaternion = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
    current_rotation = quaternion_matrix(current_quaternion)
    current_angle = np.array(tf.transformations.euler_from_quaternion(current_quaternion))
    goal_angle = np.array(tf.transformations.euler_from_quaternion(goal_quaternion))
    result_angle = np.subtract(goal_angle, current_angle)
    result_rotation = quaternion_matrix(tf.transformations.quaternion_from_euler(result_angle[0], result_angle[1], result_angle[2]))
    goal_rotation = np.dot(result_rotation[:3 , :3], current_rotation[:3 ,:3])
    #goal_rotation = quaternion_matrix(goal_quaternion)
    
    goal_matrix = np.array(goal_rotation[:3, :3])
    current_matrix = np.array(current_rotation[:3, :3])
    difference_rotation = sp.Matrix(np.subtract(goal_matrix, current_matrix))
    differnce_position = sp.Matrix([[goal.position.x - current.position.x ], [goal.position.y - current.position.y], [goal.position.z - current.position.z]])
    
    difference = sp.Matrix.hstack(difference_rotation, differnce_position)
    print(max(difference))

    return difference


def move_robot(thetas):
    global current_joint
    final_thetas = np.add(current_joint, thetas)
    msg = Float64MultiArray()
    msg.data = [0.10639782, -0.39490708,  0.08452151, -2.40419739, -0.11066075,  2.10879556, 0.10464294]
    print(final_thetas)
    move_pub.publish(msg)


def compute_ik(req):
    global current_joint, goal_pose , current_pose
    goal_pose = req.Pose
    rospy.sleep(3)
    answer = Bool()
    

    while abs(differential_error(goal_pose,current_pose)) > 0.1:
        jacobian_vectors = compute_jacobian(current_joint)
        Jacobian = [jacobian_vectors.a1, jacobian_vectors.a2, jacobian_vectors.a3, jacobian_vectors.a4, jacobian_vectors.a5, jacobian_vectors.a6, jacobian_vectors.a7,jacobian_vectors.a8,jacobian_vectors.a9,jacobian_vectors.a10,jacobian_vectors.a11,jacobian_vectors.a12]
        resultant_matrix = difference_matrix(goal_pose, current_pose)
        A_vector = extract_elements(resultant_matrix)
        incremental_thetas = compute_least_square(Jacobian,A_vector)
        #print(incremental_thetas[0])
        move_robot(incremental_thetas[0])
        rospy.sleep(5)
    answer.data = True
    return answer
    



if __name__ == '__main__':

    rospy.init_node('incremental_inverse')

    move_pub = rospy.Publisher("/joint_position_example_controller_sim/joint_command", Float64MultiArray, queue_size = 1000)
    rospy.wait_for_service("/compute_another_jacobian")
    rospy.wait_for_service("/pose_to_cartesian")

    pose_to_cartesian = rospy.ServiceProxy("/pose_to_cartesian", cartesianfrompose)
    compute_jacobian = rospy.ServiceProxy("/compute_another_jacobian", jacobian_service)
    sub = rospy.Subscriber('/joint_states', JointState, states_callback)
    service = rospy.Service("/inverse_kinematics", inverse_kinematics, compute_ik)

    rospy.spin()
    """
    init_position = [0,-0.5,0,-2.5,0,2,0]


    msg = Float64MultiArray()    
    msg.data = init_position
    pub.publish(msg)
    
    rospy.sleep(2)
    msg = Float64MultiArray()
    msg.data = result_pose

    pub.publish(msg)
    n = Jacobian.shape[1]
    r = np.linalg.matrix_rank(Jacobian)
    
    U,sigma,VH = np.linalg.svd(Jacobian, full_matrices = False)
    print(U)
    V = VH.H

    sigma_inv = np.diag(np.hstack([1/sigma[:r], np.zeros(n-r)]))
    Jacobian_plus = V * sigma_inv * U.H

    x = Jacobian_plus * b
    eps = np.linalg.norm(b - Jacobian*x)
    print(eps)
    """
    """
    a = np.array([a11theta, a21theta, a31theta, a12theta, a22theta, a32theta, a13theta])
    b = np.array([da11, da21, da31, da12, da22, da32, da13])
    print(b)
    X = np.dot(np.linalg.inv(a)*(1e-15), b.transpose())
    print(X)
    
    print(np.dot(a31theta,X))
    j=0
    #f = (0.4226 * (math.radians(3.96)*10)) + (-0.4226 * (math.radians(6.03)*10)) + (-0.4226 *(math.radians(3.39)*10))
    
    
    pub = rospy.Publisher("/joint_position_example_controller_sim/joint_command", Float64MultiArray, queue_size = 1000)

    print(goal_array)
    init_position = [0,-0.5,0,-2.5,0,2,0]

    msg = Float64MultiArray()    
    msg.data = init_position
    pub.publish(msg)
    
    rospy.sleep(2)
    msg = Float64MultiArray()
    msg.data = goal_array

    pub.publish(msg)
    """
