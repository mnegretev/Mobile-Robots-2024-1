#!/usr/bin/env python3
#
# MOBILE ROBOTS - UNAM, FI, 2024-1
# FINAL PROJECT - SIMPLE SERVICE ROBOT
# 
# Instructions:
# Write the code necessary to make the robot to perform the following possible commands:
# * Robot take the <pringles|drink> to the <table|kitchen>
# You can choose where the table and kitchen are located within the map.
# Pringles and drink are the two objects on the table used in practice 07.
# The Robot must recognize the orders using speech recognition.
# Entering the command by text or similar way is not allowed.
# The Robot must announce the progress of the action using speech synthesis,
# for example: I'm going to grab..., I'm going to navigate to ..., I arrived to..., etc.
# Publishers and suscribers to interact with the subsystems (navigation,
# vision, manipulation, speech synthesis and recognition) are already declared. 
#

import rospy
import tf
import math
import time
from std_msgs.msg import String, Float64MultiArray, Float64, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from vision_msgs.srv import *
from hri_msgs.msg import *

NAME = "Betancourt Astilla Jair"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.hypothesis[0]
    print("New command received: " + recognized_speech)

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    return obj, loc

#
# This function sends the goal articular position to the left arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the left gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal articular position to the right arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the right gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal pan-tilt angles to the head and sleeps 1 second
# to allow the head to reach the goal position. 
#
def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

#
# This function sends a linear and angular speed to the mobile base to perform
# low-level movements. The mobile base will move at the given linear-angular speeds
# during a time given by 't'
#
def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())

#
# This function publishes a global goal position. This topic is subscribed by
# pratice04 and performs path planning and tracking.
#
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

#
# This function sends a text to be synthetized.
#
def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

#
# This function calls the service for calculating inverse kinematics for left arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/la_ik_pose", InverseKinematicsPose2Pose)
    resp = clt(req_ik)
    return resp.q

#
# This function calls the service for calculating inverse kinematics for right arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik = InverseKinematicsRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/ra_ik_pose", InverseKinematicsPose2Pose)
    resp = clt(req_ik)
    return [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]

#
# Calls the service for finding object (practice 08) and returns
# the xyz coordinates of the requested object w.r.t. "realsense_link"
#
def find_object(object_name):
    clt_find_object = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObject)
    req_find_object = RecognizeObjectRequest()
    req_find_object.point_cloud = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
    req_find_object.name  = object_name
    resp = clt_find_object(req_find_object)
    return [resp.recog_object.pose.position.x, resp.recog_object.pose.position.y, resp.recog_object.pose.position.z]

#
# Transforms a point xyz expressed w.r.t. source frame to the target frame
#
def transform_point(x,y,z, source_frame, target_frame):
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = source_frame
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    obj_p = listener.transformPoint(target_frame, obj_p)
    return [obj_p.point.x, obj_p.point.y, obj_p.point.z]

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_project")
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10);
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10);
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10);
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float64, queue_size=10);
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_ik_pose')
    rospy.wait_for_service('/manipulation/ra_ik_pose')
    rospy.wait_for_service('/vision/obj_reco/recognize_object')
    print("Services are now available.")

    #
    # FINAL PROJECT 
    #
    global executing_task
    current_state = "SM_INIT"
    new_task = False
    goal = False 
    
    global initial_location
    global goal_location
    global object_name
    global current_location
    initial_location = (0.0, 0.0)
    
    while not rospy.is_shutdown():
    ##----State 0
        if current_state == "SM_INIT":
            print("State 0: " + current_state + "\n")
            print("Waiting for new task")
            say("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
            
    ##----State 1
        elif current_state == "SM_WAITING_NEW_TASK":
            print("State 1: " + current_state + "\n")
	if new_task:
		requested_object, requested_location = parse_command(recognized_speech)
		print("New task received: " + requested_object + " to  " + str(requested_location))
		say("Executing the command, " + recognized_speech)
		current_state = "SM_MOVE_HEAD"
		new_task = False
		executing_task = True
                
    ##----State 2
        elif current_state == "SM_MOVE_HEAD":
            print("State 2: " + current_state + "\n")
            say("Moving head to look at table")
            print("Moving head to look at table...")
            rospy.sleep(3)
            move_head(0, -0.9)
            if (requested_object = "pringles") or (requested_object = "drink"):
            	say("Object found")
            	current_state = "SM_FIND_OBJECT"
            else
            	say("Object not found. Return to first state")
            	print("Return to first state :(")
		current_state == "SM_INIT":
            
    ##----State 3
        elif current_state == "SM_FIND_OBJECT":
            print("State 3: " + current_state + "\n")
            print("Object found")
            # Find the requested object using the vision service
            object_pose = find_object(requested_object)

		if object_pose is not None:
		    # Update the goal location with the object's pose
		    goal_location = object_pose
		    current_state = "SM_NAVIGATE_TO_LOCATION"
		else:
		    print("Object not found")
		    say("Object not found")
		    current_state = "SM_INIT"
		    
	##----State 4
	elif current_state == "SM_NAVIGATE_TO_LOCATION":
	    print("State 4: " + current_state + "\n")
	    print("Navigating to goal location")

	    if new_task:
		object_name, new_goal_location = parse_command(recognized_speech)
		new_task = False
		goal_location = new_goal_location

	    if goal_location != current_location:
		executing_task = True

		go_to_goal_pose(goal_location)

		while not goal_reached:
		    pass

		executing_task = False
		current_state = "SM_GRASP_OBJECT"
	    else:
		print("Goal location not updated")
		current_state = "SM_INIT"
		
	##----State 5
	elif current_state == "SM_GRASP_OBJECT":
	    print("State 5: " + current_state + "\n")
	    print("Grasping object")

	    if is_object_within_reach(object_name):
		executing_task = True

		grasp_object(object_name)

		while not object_grasped:
		    pass

		executing_task = False
		current_state = "SM_TRANSPORT_OBJECT"
	    else:
		print("Object not within reach")
		current_state = "SM_INIT"
		
	##----State 6
	elif current_state == "SM_TRANSPORT_OBJECT":
	    print("State 6: " + current_state + "\n")
	    print("Transporting object")

	    if not object_grasped:
		current_state = "SM_INIT"
		return

	    transport_object(object_name)

	    while not object_transported:
		pass

	    executing_task = False
	    current_state = "SM_RELEASE_OBJECT"
	    
	##----State 7
	elif current_state == "SM_RELEASE_OBJECT":
	    print("State 7: " + current_state + "\n")
	    print("Releasing object")

	    if not object_grasped:
		current_state = "SM_INIT"
		return

	    release_object(object_name)

	    while object_grasped:
		pass

	    executing_task = False
	    current_state = "SM_RETURN_TO_INIT"
		     
	##----State 8
	elif current_state == "SM_RETURN_TO_INIT":
	    print("State 8: " + current_state + "\n")
	    print("Returning to initial position")
	    say("Returning to initial position")

	    go_to_goal_pose(initial_location)

	    while current_location != initial_location:
		pass

	    executing_task = False
	    current_state = "SM_INIT"   
	    
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

