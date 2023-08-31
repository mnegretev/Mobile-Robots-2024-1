#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# ASSIGNMENT 02 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot forwards until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

NAME = "Erik Yoel Santana Apreza"

def callback_scan(msg):
    global obstacle_detected
    n = int((msg.angle_max - msg.angle_min)/msg.angle_increment/2)
    obstacle_detected = msg.ranges[n] < 1.0
    
    return

def main():
    print("ASSIGNMENT 02 - " + NAME)
    rospy.init_node("assignment02")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    
    global obstacle_detected
    obstacle_detected = False
    while not rospy.is_shutdown():
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = 0 if obstacle_detected else 0.3
        pub_cmd_vel.publish(msg_cmd_vel)
        
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
