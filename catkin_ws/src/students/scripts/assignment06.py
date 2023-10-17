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

NAME = "JARQUIN LOPEZ DANIEL EDUARDO"

listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()

    error_a = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a

    if error_a > math.pi:
        error_a -= 2 * math.pi
    elif error_a <= -math.pi:
        error_a += 2 * math.pi
    
    v_max = 0.5
    w_max = 1.0  
    alpha = 1.0  
    beta = 1.0   
    
    v = v_max * math.exp(-error_a * error_a / alpha)
    w = w_max * (2 / (1 + math.exp(-error_a / beta)) - 1)
    
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w

    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):

    k_attr = 1.0  
    force_x = k_attr * (goal_x - robot_x)
    force_y = k_attr * (goal_y - robot_y)
    return [force_x, force_y]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):

    k_rej = 1.0
    d0 = 0.5
    total_force_x = 0
    total_force_y = 0

    for reading in laser_readings:
        distance, angle = reading
        if distance > 0:
            laser_direction_x = math.cos(robot_a + angle)
            laser_direction_y = math.sin(robot_a + angle)
            force_magnitude = k_rej * ((1/distance - 1/d0) ** 2)
            force_x = force_magnitude * laser_direction_x
            force_y = force_magnitude * laser_direction_y
            total_force_x += force_x
            total_force_y += force_y

    return [total_force_x, total_force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print("Moving to goal point " + str([goal_x, goal_y]) + " by potential fields")
    loop = rospy.Rate(20)
    global laser_readings

    epsilon = 0.5
    tolerance = 0.1
    robot_x, robot_y, robot_a = get_robot_pose(listener)

    distance_to_goal_point = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

    while distance_to_goal_point > tolerance and not rospy.is_shutdown():
        [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
        [frx, fry] = rejection_force(robot_x, robot_y, robot_a, laser_readings)
        fx = fax + frx
        fy = fay + fry

        px = robot_x - epsilon * fx
        py = robot_y - epsilon * fy

        msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, fx, fy, pub_markers)

        loop.sleep()
        robot_x, robot_y, robot_a = get_robot_pose(listener)
        distance_to_goal_point = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

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
    
