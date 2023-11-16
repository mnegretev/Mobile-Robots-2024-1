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

NAME = "López Esquivel Andrés"

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
    #
    # TODO:
    # Calculate the forward kinematics given the set of seven angles 'q'
    # You can use the following steps:
    #     H = I   # Assing to H a 4x4 identity matrix
    #     for all qi in q:
    #         H = H * Ti * Ri
    #     H = H * Ti[7]
    #     Get xyzRPY from the resulting Homogeneous Transformation 'H'
    # Where:
    #     Ti are the Homogeneous Transformations from frame i to frame i-1 when joint i is at zero position
    #     Ri are the Homogeneous Transformations with zero-translation and rotation qi around axis Wi.
    #     Ti[7] is the final Homogeneous Transformation from gripper center to joint 7.
    # Hints:
    #     Use the tft.identity_matrix() function to get the 4x4 I
    #     Use the tft.concatenate_matrices() function for multiplying Homogeneous Transformations
    #     Use the tft.rotation_matrix() matrices Ri.
    #     Use the tft.euler_from_matrix() function to get RPY from matrix H
    #     Check online documentation of these functions:
    #     http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
    #
    
    # Observations:
    #   * q is a list of seven elements (len(q) = 7)
    #   * Ti is a list of eight elements (len(Ti) = 8) where each element is a (4,4) numpy array
    #   * Wi is a list of eight elements (len(Wi) = 8) where each element is a list of three elements (numbers)
    #     that indicates direction
    #   * Ri is an homogeneous transformation with zero translation
    
    H = tft.identity_matrix()
    for i, qi in enumerate(q):
        # rotation_matrix(angle, direction, point = None) -> rotation qi around axis Wi
        Ri = tft.rotation_matrix(angle = qi, direction = Wi[i])
        # concatenate(*matrices) -> Returns concatenation of series of transformation matrices
        H = tft.concatenate_matrices(H, Ti[i], Ri)
    H = tft.concatenate_matrices(H, Ti[7])
    
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]
    
    R, P, Y = tft.euler_from_matrix(H)
    
    return numpy.asarray([x, y, z, R, P, Y])

def jacobian(q, Ti, Wi):
    delta_q = 0.000001
    #
    # TODO:
    # Calculate the Jacobian given a kinematic description Ti and Wi
    # where:
    # Ti are the Homogeneous Transformations from frame i to frame i-1 when joint i is at zero position
    # Wi are the axis of rotation of i-th joint
    # Use the numeric approximation:   f'(x) = (f(x+delta) - f(x-delta))/(2*delta)
    #
    # You can do the following steps:
    #     J = matrix of 6x7 full of zeros
    #     q_next = [q1+delta       q2        q3   ....     q7
    #                  q1       q2+delta     q3   ....     q7
    #                              ....
    #                  q1          q2        q3   ....   q7+delta]
    #     q_prev = [q1-delta       q2        q3   ....     q7
    #                  q1       q2-delta     q3   ....     q7
    #                              ....
    #                  q1          q2        q3   ....   q7-delta]
    #     FOR i = 1,..,7:
    #           i-th column of J = ( FK(i-th row of q_next) - FK(i-th row of q_prev) ) / (2*delta_q)
    #     RETURN J
    #     
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])            # J 6x7 full of zeros
    Q = q.tolist()
    q_next = [Q[:i] + [Q[i] + delta_q] + Q[i + 1:] for i in range(len(Q))] # 7 x 7
    q_prev = [Q[:i] + [Q[i] - delta_q] + Q[i + 1:] for i in range(len(Q))] # 7 x 7
    
    for i in range(len(Q)):
        FK_next = forward_kinematics(q = q_next[i], Ti = Ti, Wi = Wi) # 6 x 1
        FK_prev = forward_kinematics(q = q_prev[i], Ti = Ti, Wi = Wi) # 6 x 1
        J_col_i = (numpy.asarray(FK_next) - numpy.asarray(FK_prev)) / 2 * delta_q
        J[:,i] = J_col_i
    
    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, Ti, Wi, initial_guess):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])  # Desired configuration
    tolerance = 0.01
    max_iterations = 20
    iterations = 0
    #
    # TODO:
    # Solve the IK problem given a kinematic description (Ti, Wi) and a desired configuration.
    # where:
    # Ti are the Homogeneous Transformations from frame i to frame i-1 when joint i is at zero position
    # Wi are the axis of rotation of i-th joint
    # Use the Newton-Raphson method for root finding. (Find the roots of equation FK(q) - pd = 0)
    # You can do the following steps:
    #
    #    Set an initial guess for 'q' (given as parameter)
    #    Calculate Forward Kinematics 'p' by calling the corresponding function
    #    Calcualte error = p - pd
    #    Ensure orientation angles of error are in [-pi,pi]
    #    WHILE |error| > TOL and iterations < maximum iterations:
    #        Calculate Jacobian
    #        Update q estimation with q = q - pseudo_inverse(J)*error
    #        Ensure all angles q are in [-pi,pi]
    #        Recalculate forward kinematics p
    #        Recalculate error and ensure angles are in [-pi,pi]
    #        Increment iterations
    #    Return calculated q if maximum iterations were not exceeded
    #    Otherwise, return None
    #
    q = numpy.asarray(initial_guess)  # Initial guess
    p = forward_kinematics(q = q, Ti = Ti, Wi = Wi)
    e = p - pd
    e[3:] = (e[3:] + numpy.pi) % (2 * numpy.pi) - numpy.pi
    while numpy.linalg.norm(e) > tolerance and iterations < max_iterations:
        J = jacobian(q = q, Ti = Ti, Wi = Wi)
        q = q - numpy.dot(numpy.linalg.pinv(J), e)
        q = (q + numpy.pi) % (2 * numpy.pi) - numpy.pi
        p = forward_kinematics(q = q, Ti = Ti, Wi = Wi)
        e = p - pd
        e[3:] = (e[3:] + numpy.pi) % (2 * numpy.pi) - numpy.pi
        iterations += 1

    if iterations > max_iterations:
        print("It was not possible to calculate IK")
        return None

    return q

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
