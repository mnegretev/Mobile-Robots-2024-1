#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# PLOTTER
#
import numpy
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point, Twist
from navig_msgs.srv import SmoothPath
from navig_msgs.srv import SmoothPathResponse
import matplotlib.pyplot as plt
import numpy as np

def callback_cmd_vel(msg):
    global cmd_vel_linear_x
    cmd_vel_linear_x.append(msg.linear.x)

def main():
    global cmd_vel_linear_x
    print("PLOTTER")
    rospy.init_node("plotter", anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, callback_cmd_vel)
    cmd_vel_linear_x = []
    rospy.spin()
    t = np.linspace(0, len(cmd_vel_linear_x)*0.1, len(cmd_vel_linear_x))
    cmd_vel_linear_x = np.asarray(cmd_vel_linear_x)
    plt.title('Velocidad lineal')
    plt.plot(t, cmd_vel_linear_x)
    plt.show()

if __name__ == '__main__':
    main()
