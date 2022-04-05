#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import sys
import rospy
from kortex_driver.srv import ComputeForwardKinematics
from kortex_driver.srv import ComputeForwardKinematicsRequest
from kortex_driver.srv import ComputeForwardKinematicsResponse
from kortex_driver.msg import JointAngle, JointAngles
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse



# def computeFK(joint_angles):
#     print("entered compute fk")
#     rospy.wait_for_service('compute_forward_kinematics')
#     try:
#         print("trying...")
#         forward_k = rospy.ServiceProxy("/compute_fk", GetPositionFK)
#         print("got service handle")
#         res = forward_k(joint_angles)
#         print("called service")
#         return res.output

#     except rospy.ServiceException(e):
#         print("Service called failed: %s"%e)

def main():
    # a = np.uint32(0)
    # b = np.uint32(1)
    # c = np.uint32(2)
    # d = np.uint32(3)
    # e = np.uint32(4)
    # f = np.uint32(5)
    # g = np.uint32(6)
    # joint_id = [a,b,c,d,e,f,g]
    # joint_angles = JointAngles()
    # joint_angle_list = []
    # for i in range(7):

    #     joint_angle = JointAngle()
    #     joint_angle.joint_identifier = np.uint(i)
    #     joint_angle.value = 1
    #     joint_angle_list.append(joint_angle)

    # joint_angles.joint_angles = joint_angle_list
    # print("Before compute fk")
    # output = computeFK(joint_angles)
    # print(output)
    
    compute_fk_srv = rospy.ServiceProxy("/my_gen3/compute_fk", GetPositionFK)
    print("waiting for service")
    rospy.wait_for_service("/my_gen3/compute_fk")
    print("found service")
    fk_request = GetPositionFKRequest()
    fk_request.header.frame_id = "base_link"
    # fk_request.header.stamp = rospy.Time.now()
    fk_request.fk_link_names = ['end_effector_link']
    fk_request.robot_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    fk_request.robot_state.joint_state.position = [4.3277119443629886e-06, 0.260162077999877, 3.140033090338544, -2.2700807449440203, -2.3753016797911641e-07, 0.9598476470557813, 1.5699991584482955]
    
    output = compute_fk_srv(fk_request)
    
    print((output))

main()
