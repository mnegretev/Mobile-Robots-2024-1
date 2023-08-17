#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# PRACTICE 03 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#
import numpy
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from navig_msgs.srv import SmoothPath
from navig_msgs.srv import SmoothPathResponse

NAME = "FULL NAME"

msg_smooth_path = Path()

def smooth_path(Q, alpha, beta):
    print("Smoothing path with params: " + str([alpha,beta]))
    #
    # TODO:
    # Write the code to smooth the path Q, using the gradient descend algorithm,
    # and return a new smoothed path P.
    # Path is composed of a set of points [x,y] as follows:
    # [[x0,y0], [x1,y1], ..., [xn,ym]].
    # The smoothed path must have the same shape.
    # Return the smoothed path.
    #
    P = numpy.copy(Q)
    tol     = 0.00001                   
    nabla   = numpy.full(Q.shape, float("inf"))
    epsilon = 0.1                       
    steps   = 0


    
    print("Path smoothed succesfully after " + str(steps) + " iterations")
    return P

def callback_smooth_path(req):
    global alpha, beta
    P = smooth_path(numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in req.path.poses]), alpha, beta)
    msg_smooth_path.poses = []
    for i in range(len(req.path.poses)):
        msg_smooth_path.poses.append(PoseStamped(pose=Pose(position=Point(x=P[i,0],y=P[i,1]))))
    return SmoothPathResponse(smooth_path=msg_smooth_path)

def main():
    global alpha, beta
    print("PRACTICE 03 - " + NAME)
    rospy.init_node("practice03", anonymous=True)
    rospy.Service('/path_planning/smooth_path', SmoothPath, callback_smooth_path)
    pub_path = rospy.Publisher('/path_planning/smooth_path', Path, queue_size=10)
    loop = rospy.Rate(1)
    msg_smooth_path.header.frame_id = "map"
    alpha = rospy.get_param("~alpha", 0.9)
    beta  = rospy.get_param("~beta" , 0.1)
    while not rospy.is_shutdown():
        pub_path.publish(msg_smooth_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
