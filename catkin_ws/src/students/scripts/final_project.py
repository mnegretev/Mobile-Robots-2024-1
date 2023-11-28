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

NAME = "Rodríguez Jiménez Bruno A"

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
    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "E0_START"
    requested_object   = ""
    requested_location = [0,0]

    # Bucle principal
    while not rospy.is_shutdown():
         # Máquina de estados
        if current_state == "E0_START": 			# Estado 0: Inicio de la máquina
            print("E0:INICIANDO MÀQUINA DE ESTADOS")
            if(new_task == True):
                new_task = False
                current_state = "E1_HELLO"

        elif current_state == "E1_HELLO": 		# Estado 1: Saludo del robot con voz
            print("E1: SALUDO")
            say("Hello world")
            current_state = "E2_RECOR"

        elif current_state == "E2_RECOR":		# Estado 2: Reconocimiento de orden
            print("E2: RECONOCIMIENTO DE OBJETO")
            say("Robot is recognizing object")
            obj,loc = parse_command(recognized_speech) 	#Reconocimiento de voz
            print("   Objeto a agarrar: "+ obj) 
            print("   Localizaciòn: ["+ str(loc[0]) + "," + str(loc[1])+ "]")
            current_state = "E3_HEAD"

        elif current_state == "E3_HEAD":		# Estado 3: Posicionamiento de cabeza
            print("E3: POSICIONAMIENTO DE CABEZA")
            move_head(0,-0.9)
            current_state = "E4_FIND_OBJ"

        elif current_state == "E4_FIND_OBJ":		# Estado 4: Búsqueda de objeto
            print("E4: BUSCANDO OBJETO")
            say("Finding object")
            x_obj, y_obj, z_obj = find_object(obj)		
            print("   Centroide de objeto: ["+str(x_obj)+","+str(y_obj)+","+ str(z_obj)+"]")
            current_state = "E5_TRANSFORM"

        elif current_state == "E5_TRANSFORM":		# Estado 5: Transformación de coordenadas
         print("E5: TRANSFORMACIÒN DE COORDENADAS")
         if obj == "pringles":
                xt, yt, zt = transform_point(x_obj, y_obj, z_obj,"realsense_link","shoulders_left_link")
         else:
                xt, yt, zt = transform_point(x_obj, y_obj, z_obj,"realsense_link","shoulders_right_link")
         print("   Coordenadas transformadas: ["+ str(xt) + str(yt) + str(zt))
         current_state = "E6_CIN_INV"

        elif current_state == "E6_CIN_INV":		# Estado 6: Cinemática inversa
         print("E6: CINEMÀTICA INVERSA")
         if obj == "pringles":
                q = calculate_inverse_kinematics_left(xt,yt,zt,3,-1.57,-3)
         else:
                q = calculate_inverse_kinematics_right(xt,yt+.001,zt,3,-1.57,-3)
         print(q)
         current_state = "E7_ARM_INI"

        elif current_state == "E7_ARM_INI":		# Estado 7: Brazo a posición inicial
         print("E7: BRAZO A POSICION INICIAL")
         if obj == "pringles":    
          print("   Brazo izquierdo a posicion inicial")
          move_left_arm(-0.5,0,0,2.3,0,0,0)
         else:
          print("   Brazo derecho a posicion inicial")
          move_right_arm(-0.5,0,0,2.3,0,0,0)
          move_right_arm(-0.4,0,0,2.3,.5,0,0)
         current_state = "E8_ARM_OBJ"

        elif current_state == "E8_ARM_OBJ":		# Estado 8: Brazo a objeto
            print("E8: BRAZO A OBJETO")
            say("Robot is moving its arm")
            if obj == "pringles":
             move_left_gripper(0.5)
             move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            else:
             move_right_gripper(0.5)
             move_right_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            current_state = "E9_CLOSE"
        
        elif current_state == "E9_CLOSE":		# Estado 9: Cerrar mano
            print("E9: CERRANDO MANO")
            if obj == "pringles":
             move_left_arm(-0,0,0,1.7,0,0,0)
             move_left_gripper(-0.5)
             move_left_arm(0,0,0,3,0,0,-1)
            else:
             #move_right_arm(-0,0,0,1.7,0,0,0)
             move_right_gripper(-0.5)
             move_right_arm(0,0,0,3,0,0,-0.8)
             say("I got it")
            current_state = "E10_TO_GOAL"

                
        elif current_state == "E10_TO_GOAL":		# Estado 10: Moverse a meta
         print("E10: DIRIGIRSE A META ")
         go_to_goal_pose(loc[0],loc[1])
         if goal_reached:
           move_left_gripper(0.5)
           print("   Objeto depositado")
           goal_reached = False
           current_state = "E11_TO_HOME"
           
        elif current_state == "E11_TO_HOME" :		# Estado 11: Regresar a inicio
         print("E11: REGRESAR A INICIO")
         go_to_goal_pose(2,5.86)
         if goal_reached == True:
          print("Fin")
          move_base(0,1,15)
          move_left_gripper(0)
          current_state = "E12_END"
         

          

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
   
