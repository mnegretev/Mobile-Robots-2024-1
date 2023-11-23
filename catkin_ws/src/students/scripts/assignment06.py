#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# ASSIGNMENT 06 - OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants d0, eta and zeta to get a smooth movement. 
#

import rospy
import tf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "Romero Trujillo Jerson Gerardo"

listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds.
    # v_max, w_max, alpha and beta, are design constants.
    # Maximum linear and angular speeds must be 0.5 and 1.0 respectively.
    # Store the resulting v and w in the Twist message 'cmd_vel'
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    alpha = 0.1
    beta = 0.1
    v_max = 0.5
    w_max = 1
    num_pi = math.pi
    error_a = math.atan2(goal_y-robot_y,goal_x-robot_x)-robot_a
    if error_a >= num_pi or error_a < -num_pi:
    	error_a = (error_a+ num_pi)%(2*num_pi)-num_pi
    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Calculate the attraction force, given the robot and goal positions.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force w.r.t. map.
    #
    zeta = 1.0
    force_x, force_y = robot_x - goal_x, robot_y - goal_y
    mag = math.sqrt(force_x**2 + force_y**2)
    return [zeta*force_x/mag, zeta*force_y/mag] if mag != 0 else [0, 0]
    

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    #
    # TODO:
    # Calculate the total rejection force given by the average
    # of the rejection forces caused by each laser reading.
    # laser_readings is an array where each element is a tuple [distance, angle]
    # both measured w.r.t. ROBOT's FRAME. 
    # See lecture notes for equations to calculate rejection forces.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting rejection force w.r.t. MAP.
    # WARNING: Some laser readings could have distance=0 due to simulated reading errors. 
    #
    d0 = 1.1
    eta = 3.5
    [force_x, force_y] = [0, 0]
    for [d, theta] in laser_readings:
    	if (d >= d0 or d <= 0):
    		mag = 0
    	else:
    		mag = eta*math.sqrt(1/d - 1/d0)
    	force_x += mag*math.cos(robot_a + theta)
    	force_y += mag*math.sin(robot_a + theta)
    return [force_x/len(laser_readings), force_y/len(laser_readings)]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print("Moving to goal point " + str([goal_x, goal_y]) + " by potential fields"    )
    loop = rospy.Rate(20)
    global laser_readings

    epsilon = 0.5
    tolerance = 0.1
    robot_x, robot_y, robot_a = get_robot_pose(listener)
    distance_to_goal_point = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    while distance_to_goal_point > tolerance and not rospy.is_shutdown():
    	[fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
    	[frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings)
    	[fx, fy] = [fax+frx, fay+fry]
    	[px, py] = [robot_x - epsilon*fx, robot_y - epsilon*fy]
    	msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
    	pub_cmd_vel.publish(msg_cmd_vel)
    	draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, fx, fy, pub_markers)
    	loop.sleep()
    	robot_x, robot_y, robot_a = get_robot_pose(listener)
    	distance_to_goal_point = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    pub_cmd_vel.publish(Twist())

    print("Goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def callback_scan(msg):
    global laser_readings
    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]

def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    hdr = Header(frame_id="map", stamp=rospy.Time.now())
    mrk = Marker(header=hdr, ns="pot_fields", id=id, type=Marker.ARROW, action=Marker.ADD)
    mrk.pose.orientation.w = 1
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point(x=robot_x, y=robot_y))
    mrk.points.append(Point(x=(robot_x - force_x), y=(robot_y - force_y)))
    return mrk

def main():
    global listener, pub_cmd_vel, pub_markers
    print("ASSIGNMENT 06 - " + NAME)
    rospy.init_node("assignment06")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_pot_fields_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
