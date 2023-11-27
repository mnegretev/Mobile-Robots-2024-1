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

from manip_msgs.srv import *

NAME = "FLORES_RIVAS, MEZA_MARTINEZ, RODRIGUEZ_FUENTES"

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
    time.sleep(1.0)

#
# This function sends the goal angular position to the left gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.5)

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
    time.sleep(1.0)

#
# This function sends the goal angular position to the right gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.5)

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
    return goal_pose

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
def calculate_inverse_kinematics_left(x, y, z, roll, pitch, yaw):
    req_ik = InverseKinematicsPose2PoseRequest()   
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
def calculate_inverse_kinematics_right(x, y, z, roll, pitch, yaw):
    req_ik = InverseKinematicsPose2PoseRequest()   
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/ra_ik_pose", InverseKinematicsPose2Pose)
    resp = clt(req_ik)
    return resp.q

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

def location(object):
    if object=='pringles':
        target = 'shoulders_left_link'
    else:
        target = 'shoulders_right_link'
    x, y, z = find_object(object) #Coordenadas w.r.t
    x, y, z = transform_point(x, y, z, "realsense_link", target)#Pasar coordenadas al target
    if object=='pringles':
        maxi = 0
        while x > 0.62 or x < 0.58 or z < -0.46:
            x, y, z = find_object(object) #Coordenadas w.r.t
            x, y, z = transform_point(x, y, z, "realsense_link", target)
            maxi += 1
            if maxi == 10:
                x = 0.60
                z = -0.38
    return (x,y,z)
    
def take_object(object, x, y, z):
    # move_base(0.1, 0.0, 1.0)#Que se acerque a la mesa un poco
    print(f"Object: {object}")
    if object=='pringles':
        move_left_arm(-1.2,0.2,0.0,1.9,0.0,1.6,0.0) #Mover a la posicion prepare left
        q = calculate_inverse_kinematics_left(x-0.02,y+0.03,z+0.15,0,-1.3,0)#5 cm alejado del centroide, pitch para que alcance
        print(f"{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]},{q[6]}")
        move_left_gripper(0.3)
        move_base(-0.1, 0.0, 1.0)
        move_left_arm(q[0]/2.0,0.2,0.0,1.9,0.0,1.6,0.0)
        move_left_arm(q[0]/2.0,q[1]/2.0,0.0,1.9,0.0,1.6,0.0)
        move_left_arm(q[0]/2.0,q[1]/2.0,q[2]/2.0,1.9,0.0,1.6,0.0)
        move_left_arm(q[0]/2.0,q[1]/2.0,q[2]/2.0,1.9,0.0,0.0,0.0)
        move_left_arm(q[0]/1.5,q[1],q[2]/2.0,1.9,0.0,q[5]/2.0,0.0)
        move_left_arm(q[0]/1.5,q[1],q[2]/1.5,1.9,0.0,q[5]/2.0,0.0)
        
        move_base(0.2, 0.0, 0.8)
        move_base(0.1, 0.0, 0.6)
        
        move_left_arm(q[0]/1.5,q[1],q[2],1.9,0.0,q[5]/2.0,0.0)
        move_left_arm(q[0]/1.5,q[1],q[2],1.9,0.0,q[5]/1.6,q[6])
        move_left_arm(q[0]/1.5,q[1],q[2],q[3],0.0,q[5]/1.6,q[6])
        move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5]/1.6,q[6])
        move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
        
        rospy.sleep(2)#Que espere brevemente antes de chocar
        move_left_gripper(-0.3)
        move_left_arm(q[0]+0.4,q[1],q[2],q[3],q[4],q[5],q[6])
        move_base(-0.2,0,2.0)#Retroceder a una posicion segura una vez tomado el objeto
        move_left_arm(-1.2,0.2,0.0,1.9,0.0,1.6,0.0)
        
    else:
        move_right_arm(-1.2,-0.2,0.0,1.9,1.6,0.0,0.0) #Mover a la posicion prepare right
        print(f"({x} {y} {z})")
        q = calculate_inverse_kinematics_right(x+0.08,y,z+0.17,0,-1.3,0)
        print(f"{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]},{q[6]}")
        move_right_gripper(0.3)
        
        move_right_arm(q[0]/2.0,-0.2,0.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/2.0,q[1]/2.0,0.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/2.0,q[1]/2.0,q[2]/2.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/2.0,q[1]/2.0,q[2]/2.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/1.5,q[1],q[2]/2.0,1.9,q[4]/2.0,0.0,0.0)
        move_right_arm(q[0]/1.5,q[1],q[2],1.9,q[4]/2.0,0.0,0.0)
        
        move_base(0.2, 0.0, 0.8)
        move_base(0.1, 0.0, 1.0)
        
        move_right_arm(q[0]/1.5,q[1],q[2],1.9,q[4]/2.0,0.0,0.0)
        move_right_arm(q[0]/1.5,q[1],q[2],1.9,q[4]/1.6,0.0,q[6])
        move_right_arm(q[0]/1.5,q[1],q[2],q[3],q[4]/1.6,q[5]/2.0,q[6])
        move_right_arm(q[0],q[1],q[2],q[3],q[4]/1.6,q[5],q[6])
        move_right_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
        # move_base(0.1, 0.0, 1.0)
        rospy.sleep(2)#Que espere brevemente antes de chocar
        move_right_gripper(-0.3)#Creo que con este valor ya alcanza a agarrar cualquier objeto
        move_right_arm(q[0]+0.3,q[1],q[2],q[3],q[4]+0.3,q[5],q[6])
        move_base(-0.2,0,2.0)#Retroceder a una posicion segura una vez tomado el objeto
        move_right_arm(-1.2,-0.2,0.0,1.9,1.6,0.0,0.0)

def orientation(goal_pose):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 3.26
    goal_pose.pose.position.y = 6.30
    goal_pose.pose.orientation.z = -0.71
    goal_pose.pose.orientation.w = 0.71
    pubGoalPose.publish(goal_pose)
    
def set2zero(goal_pose):#Calibrar
    orientation(goal_pose)
    move_left_arm(0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    move_left_gripper(0)
    move_right_arm(0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    move_right_gripper(0)
    move_head(0, 0)
    
def delivery(object):
    
    
    if object == "pringles":
        move_left_arm(0.5857831878465416,0.0,0.0,0.0,0.0,1.6,0.0)
        move_left_gripper(0.3)
    else:
        move_right_arm(0.5857831878465416,0.0,0.0,0.0,1.6,0.0,0.0)
        move_right_gripper(0.3)
    
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
    executing_task = False
    current_state = "SM_INIT"
    new_task = False
    while not rospy.is_shutdown():
        if current_state == "SM_INIT":
            print("################################# SM_INIT #################################")
            print("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
            
        elif current_state == "SM_WAITING_NEW_TASK":
            print("############################ SM_WAITING_NEW_TASK ##########################")
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("New task received: " + requested_object + " to  " + str(requested_location))
                say("Executing the command, " + recognized_speech)
                current_state = "SM_CALIBRATION"
                new_task = False
                executing_task = True
                time.sleep(1.0)
                
        elif current_state == "SM_CALIBRATION":
            print("############################## SM_CALIBRATION #############################")
            goal_pose = go_to_goal_pose(3.26, 6.3)
            set2zero(goal_pose)
            print(goal_pose)
            if goal_pose.pose.orientation.z >= -1 and goal_pose.pose.orientation.z <= 0:
                say("Calibration Done")
                current_state = "SM_MOVE_HEAD"
                time.sleep(1.0)
                
        elif current_state == "SM_MOVE_HEAD":
            print("############################### SM_MOVE_HEAD ##############################")
            if requested_object == 'pringles':
                move_head(0.0, -1.0)
            else:
                move_head(0.0, -0.6)
            say("Moving head to look at table...")
            current_state = "SM_FIND_OBJECT"
            time.sleep(1.0)

        elif current_state == "SM_FIND_OBJECT":
            print("############################### SM_FIND_OBJECT #############################")
            say("Searching for, " + requested_object)
            time.sleep(1.0)
            if requested_object == 'pringles':
                move_base(0.2, 0.0, 2.2) #Que se acerque a la mesa un poco
                x, y, z = location(requested_object)
            else:
                print(requested_object)
                x, y, z = location(requested_object)
                move_base(0.2, 0.0, 2.5)
                x = x - 0.3
            print(f"Object found at: ({x}, {y}, {z})")
            say("I found the object")
            current_state = "SM_TAKE_OBJECT"
            time.sleep(1.0)
            
        elif current_state == "SM_TAKE_OBJECT":
            print("############################### SM_TAKE_OBJECT #############################")
            take_object(requested_object, x, y, z)
            print("Object succesfully taken")
            say("I got the object")
            
            current_state = "SM_NAVIGATE"
            time.sleep(1.0)
            
        elif current_state == "SM_NAVIGATE":
            print("################################ SM_NAVIGATE ###############################")
            goal_reached = False
            xf, yf = requested_location[0], requested_location[1]
            go_to_goal_pose(xf, yf)
            print("Navigation in process...")
            say("I will deliver the object")
            current_state = "SM_GOAL_REACHED"
            time.sleep(1.0)
            
        elif current_state == "SM_GOAL_REACHED":
            print("############################## SM_GOAL_REACHED #############################")
            if goal_reached:
                say("I have reached the goal")
                current_state = "SM_DELIVER"
                goal_reached = False
                time.sleep(1.0)
            
        elif current_state == "SM_DELIVER":
            print("################################# SM_DELIVER ###############################")
            delivery(requested_object)
            say("I delivered the object")
            current_state = "SM_RETURN"
            time.sleep(1.0)
            
        elif current_state == "SM_RETURN":
            print("################################# SM_RETURN ###############################")
            goal_pose = go_to_goal_pose(3.26, 6.3)
            set2zero(goal_pose)
            if goal_reached and goal_pose.pose.orientation.z >= -1 and goal_pose.pose.orientation.z <= 0:
                say("Safely returned to home")
                executing_task = False
                goal_reached = False
                current_state = "SM_WAITING_NEW_TASK"
                time.sleep(1.0)
            
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
