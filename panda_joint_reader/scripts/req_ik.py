#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from franka_msgs.srv import inverse_kinematics

if __name__ == '__main__':
    pose = Pose()
    rospy.init_node('req_ik')
    rospy.wait_for_service('/inverse_kinematics')
    inv_kin = rospy.ServiceProxy('/inverse_kinematics',inverse_kinematics)

    pose.position.x = 0.237835954989
    pose.position.y = -0.0691358422275
    pose.position.z = 0.337515215212

    pose.orientation.x = -0.992487908011
    pose.orientation.y = 0.0650138572723
    pose.orientation.z = 0.0444670988479
    pose.orientation.w = 0.0936142507055

    try:
        Movement = inv_kin(pose)
        print(Movement)

    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
        pass

    

