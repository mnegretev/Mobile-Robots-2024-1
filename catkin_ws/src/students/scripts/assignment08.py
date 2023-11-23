#!/usr/bin/env python3
#
# MOBILE ROBOTS - UNAM, FI, 2024-1
# ASSIGNMENT 08 - INVERSE KINEMATICS
#
# Instructions:
# Calculate the inverse kinematics for both manipulators (left and right) given the
# URDF descriptions and a desired configuration. Solve the inverse kinematics using
# the Newton-Raphson method for root finding.
# Modify only sections marked with the 'TODO' comment
#

import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from manip_msgs.srv import *

NAME = "JARQUIN LOPEZ DANIEL EDUARDO"

def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = {'left': [None for i in range(8)], 'right': [None for i in range(8)]}
    transforms = {'left':[], 'right':[]}
    for joint in robot_model.joints:
        for i in range(1,8):
            joints['left' ][i-1] = joint if joint.name == ('la_'+str(i)+'_joint') else joints['left' ][i-1]
            joints['right'][i-1] = joint if joint.name == ('ra_'+str(i)+'_joint') else joints['right'][i-1]
        joints['left' ][7] = joint if joint.name == 'la_grip_center_joint' else joints['left' ][7]
        joints['right'][7] = joint if joint.name == 'ra_grip_center_joint' else joints['right'][7]
    for joint in joints['left']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['left'].append(tft.concatenate_matrices(T,R))
    for joint in joints['right']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['right'].append(tft.concatenate_matrices(T,R))
    def forward_kinematics(q, Ti, Wi):
    H = tft.identity_matrix()  # 4x4 identity matrix
    for i, qi in enumerate(q):
        Ri = tft.rotation_matrix(qi, Wi[i])
        H = numpy.dot(H, numpy.dot(Ti[i], Ri))
    H = numpy.dot(H, Ti[7])
    xyz_rpy = tft.euler_from_matrix(H, 'sxyz')
    return numpy.concatenate((H[:3, 3], xyz_rpy))

def jacobian(q, Ti, Wi):
    delta_q = 0.000001
    J = numpy.zeros((6, 7))

    for i in range(7):
        q_next = q.copy()
        q_next[i] += delta_q
        p_next = forward_kinematics(q_next, Ti, Wi)

        q_prev = q.copy()
        q_prev[i] -= delta_q
        p_prev = forward_kinematics(q_prev, Ti, Wi)

        J[:, i] = (p_next - p_prev) / (2 * delta_q)

    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, Ti, Wi, initial_guess):
    pd = numpy.asarray([x, y, z, roll, pitch, yaw])  # Desired configuration
    tolerance = 0.01
    max_iterations = 20
    iterations = 0

    # Set an initial guess for 'q'
    q = numpy.asarray(initial_guess)

    while iterations < max_iterations:
        p = forward_kinematics(q, Ti, Wi)
        error = p - pd
        error[3:6] = numpy.arctan2(numpy.sin(error[3:6]), numpy.cos(error[3:6]))
        if numpy.linalg.norm(error) < tolerance:
            return q
        J = jacobian(q, Ti, Wi)
        q = q - numpy.dot(numpy.linalg.pinv(J), error)
        q = numpy.mod(q + numpy.pi, 2 * numpy.pi) - numpy.pi
        p = forward_kinematics(q, Ti, Wi)
        error = p - pd
        error[3:6] = numpy.arctan2(numpy.sin(error[3:6]), numpy.cos(error[3:6]))

        iterations += 1

    return None


def callback_la_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]
    initial_guess = rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, 5.0).data
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi, initial_guess)
    if q is None:
        return None
    resp = InverseKinematicsPose2PoseResponse()
    resp.q = q
    return resp

def callback_ra_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]
    initial_guess = rospy.wait_for_message("/hardware/right_arm/current_pose", Float64MultiArray, 5.0).data
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi, initial_guess)
    if q is None:
        return False
    resp = InverseKinematicsPose2PoseResponse()
    resp.q = q
    return resp

def callback_la_fk(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]
    x = forward_kinematics([req.q[0], req.q[1], req.q[2], req.q[3], req.q[4], req.q[5], req.q[6]], Ti, Wi)
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def callback_ra_fk(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    x = forward_kinematics([req.q[0], req.q[1], req.q[2], req.q[3], req.q[4], req.q[5], req.q[6]], Ti, Wi)
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("ASSIGNMENT 08" + NAME)
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/la_ik_pose", InverseKinematicsPose2Pose, callback_la_ik_for_pose)
    rospy.Service("/manipulation/ra_ik_pose", InverseKinematicsPose2Pose, callback_ra_ik_for_pose)
    rospy.Service("/manipulation/la_forward_kinematics", ForwardKinematics, callback_la_fk)
    rospy.Service("/manipulation/ra_forward_kinematics", ForwardKinematics, callback_ra_fk)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
