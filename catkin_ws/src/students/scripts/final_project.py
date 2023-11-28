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

from pickle import TRUE
import rospy
import tf
import tf2_ros
import math
import time
import geometry_msgs.msg
from std_msgs.msg import String, Float64MultiArray, Float64, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from vision_msgs.srv import *
from hri_msgs.msg import *

from manip_msgs.srv import *

NAME = "Guillen Castillo Jorge Luis"

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
def calculate_inverse_kinematics_left(x, y, z, roll, pitch, yaw):
    req_ik = InverseKinematicsPose2PoseRequest()   #######################
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/la_ik_pose", InverseKinematicsPose2Pose)
    resp = clt(req_ik)
    return resp.q   #######################

#
# This function calls the service for calculating inverse kinematics for right arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_right(x, y, z, roll, pitch, yaw, timeout=None):
    req_ik = InverseKinematicsPose2PoseRequest()   #######################
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/ra_ik_pose", InverseKinematicsPose2Pose)
    resp = clt(req_ik)
    return resp.q   #######################

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
def transform_point(listener, x, y, z, source_frame, target_frame):
    try:
        # Wait for the transformation
        listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))

        # Transform the point
        point_in = geometry_msgs.msg.PointStamped()
        point_in.header.frame_id = source_frame
        point_in.header.stamp = listener.getLatestCommonTime(target_frame, source_frame)
        point_in.point.x = x
        point_in.point.y = y
        point_in.point.z = z

        point_out = listener.transformPoint(target_frame, point_in)

        return point_out.point.x, point_out.point.y, point_out.point.z
    except tf2_ros.TransformException as e:
        print(f"Transform failed: {e}")
        return 0, 0, 0  # Return a default value or handle the error as neede

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
                print("Max iterations reached")
    return (x,y,z)
    
def take_object(object, x, y, z):
    # move_base(0.1, 0.0, 1.0)#Que se acerque a la mesa un poco
    print(f"Object: {object}")
    if object=='pringles':
        move_left_arm(-1.2,0.2,0.0,1.9,0.0,1.6,0.0)#Mover a la posicion prepare, es diferente en left y right arm
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
        move_base(0.1, 0.0, 0.8)
        
        move_left_arm(q[0]/1.5,q[1],q[2],1.9,0.0,q[5]/2.0,0.0)
        move_left_arm(q[0]/1.5,q[1],q[2],1.9,0.0,q[5]/1.6,q[6])
        move_left_arm(q[0]/1.5,q[1],q[2],q[3],0.0,q[5]/1.6,q[6])
        move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5]/1.6,q[6])
        move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
        
        rospy.sleep(2)#Que espere brevemente antes de chocar
        move_left_gripper(-0.3)
        move_left_arm(q[0]+0.4,q[1],q[2],q[3],q[4],q[5],q[6])
        move_base(-0.2,0,2.0)#Retroceder a una posicion segura una vez tomado el objeto
    else:
        move_right_arm(-1.2,-0.2,0.0,1.9,1.6,0.0,0.0)#Lo mismo de arriba, pero con los valores de right arm
        print(f"({x} {y} {z})")
        q = calculate_inverse_kinematics_right(x+0.05,y,z+0.17,0,-1.3,0)
        print(f"{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]},{q[6]}")
        move_right_gripper(0.3)
        # move_base(-0.1, 0.0, 1.0)
        move_right_arm(q[0]/2.0,-0.2,0.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/2.0,q[1]/2.0,0.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/2.0,q[1]/2.0,q[2]/2.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/2.0,q[1]/2.0,q[2]/2.0,1.9,1.6,0.0,0.0)
        move_right_arm(q[0]/1.5,q[1],q[2]/2.0,1.9,q[4]/2.0,0.0,0.0)
        move_right_arm(q[0]/1.5,q[1],q[2],1.9,q[4]/2.0,0.0,0.0)
        
        move_base(0.2, 0.0, 0.8)
        move_base(0.1, 0.0, 0.8)
        
        move_right_arm(q[0]/1.5,q[1],q[2],1.9,q[4]/2.0,0.0,0.0)
        print(" ######################### 1 #########################")
        move_right_arm(q[0]/1.5,q[1],q[2],1.9,q[4]/1.6,0.0,q[6])
        print(" ######################### 2 #########################")
        move_right_arm(q[0]/1.5,q[1],q[2],q[3],q[4]/1.6,q[5]/2.0,q[6])
        print(" ######################### 3 #########################")
        move_right_arm(q[0],q[1],q[2],q[3],q[4]/1.6,q[5],q[6])
        print(" ######################### 4 #########################")
        move_right_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
        print(" ######################### 5 #########################")
        # move_base(0.1, 0.0, 1.0)
        rospy.sleep(2)#Que espere brevemente antes de chocar
        move_right_gripper(-0.3)#Creo que con este valor ya alcanza a agarrar cualquier objeto
        move_right_arm(q[0]+0.3,q[1],q[2],q[3],q[4]+0.3,q[5],q[6])
        move_base(-0.2,0,2.0)#Retroceder a una posicion segura una vez tomado el objeto

def walking(ubication): #Navegar hasta la requested location
    pass
    

def finish_line(goal_ubication,current_loc):#Alcanzar el objetivo
    #Saber que llegamos y entregar el objeto
    pass
    
def main():
    global new_task, recognized_speech, executing_task, goal
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    print("VERSION FINAL")
    rospy.init_node("final_project")
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal', Bool, callback_goal_reached)
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
    goal = False
    while not rospy.is_shutdown():
        # Estado 0
        if current_state == "SM_INIT":
            print("ESTADO 0: "+current_state+"  ºººººº")
            print("Waiting for a new task...")
            current_state = "SM_WAITING_NEW_TASK"


        # Estado 1
        elif current_state == "SM_WAITING_NEW_TASK":
            print("ESTADO 1: "+current_state+"  ºººººº")
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                new_task = False
                executing_task = True
                current_state = "SM_MOVE_HEAD"
                print(" >>> New task received: " + requested_object + " to  " + str(requested_location))
                say("Executing the command, " + recognized_speech)
                rospy.sleep(5)


         # Estado 2
        elif current_state == "SM_MOVE_HEAD":
            print("ESTADO 2: "+current_state+"  ºººººº")
            print(" >>> Moving head to look at table...")
            say("Moving head to look at table")
            rospy.sleep(3)
            move_head(0, -0.9)
            current_state = "SM_MOVE_TO_OBJECTS"


        # Estado 3
        elif current_state == "SM_MOVE_TO_OBJECTS":
            print("ESTADO 3: "+current_state+"  ºººººº")
            print(" >>> I am adjusting my arm to take the object")
            say("I am adjusting my arm to take the object.")
            rospy.sleep(3)
            if requested_object == "pringles":
                move_left_arm(-1.6, 0.2, 0.0, 1.8, 0.0, 1.3, 0.0)
                print(" >>> I am moving towards the object")
                say("I am moving towards the object")
                rospy.sleep(3)
                move_base(5.0, 0.0, 3.9)
            else:
                move_right_arm(-1.6, -0.2, 0.0, 1.7, 1.2, 0.0, 0.0)
                print(" >>> I am moving towards the object")
                say("I am moving towards the object")
                rospy.sleep(3)
                move_base(5.0, 0.0, 5.1)
            print(" >>> I reached the object")
            say("I reached the object")
            rospy.sleep(5)
            current_state = "SM_RECOGNIZE_OBJECT"


        # Estado 4
        elif current_state == "SM_RECOGNIZE_OBJECT":
            print("ESTADO 4: "+current_state+"  ºººººº") 
            print(" >>> I am calculating the coordinates of the object")
            say("I am calculating the coordinates of the object")
            rospy.sleep(3)
            # Debug print to check the value of requested_object
            print("Requested Object:", requested_object)
           
            target = "shoulders_left_link" if requested_object == "pringles" else "shoulders_right_link"
            x1, y1, z1 = find_object(object_name = requested_object)
           
            # Debug print to check the values obtained from find_object
            print("Object Coordinates (raw):", x1, y1, z1)
            
            x, y, z = transform_point(listener, x1, y1, z1, "realsense_link", target)
            print(" >>> I got the correct coordinates of the object")
            say("I got the correct coordinates of the object")
            rospy.sleep(3)
            current_state = "SM_PREPARE_TAKE"



        # Estado 5
        elif current_state == "SM_PREPARE_TAKE":
            print("ESTADO 5: "+current_state+"  ºººººº")
            print(" >>> I am preparing my arm to take the object")
            say("I am preparing my arm to take the object.")
            rospy.sleep(3)
            move_head(pan = 0.0, tilt = 0.0)
            if requested_object == "pringles":
                move_left_arm(0.0730, 0.2440, -0.0327, 2.1538, 0.0888, -0.3717, 0.0646)
                move_left_gripper(0.5)
            else:
                move_right_arm(0.0, -0.2, 0.1, 1.4,  0.5, 0.0, 0.0)
                move_right_gripper(0.4)
            current_state = "SM_TAKE_OBJECT"


       # Estado 6
        elif current_state == "SM_TAKE_OBJECT":
            print("ESTADO 6: "+current_state+"  ºººººº")
            print(" >>> I am ready to take the object")
            say("I am ready to take the object")
            rospy.sleep(3)
            if requested_object == "pringles":
                try:
                    q = calculate_inverse_kinematics_left(x + 0.08, y, z + 0.07, 0, -1.6, 0)
                    move_left_arm(q[0], q[1], q[2], q[3], q[4], q[5], q[6])
                    move_left_gripper(-0.35)
                    move_left_arm(q[0], q[1], q[2], q[3] + 0.3, q[4], q[5], q[6])
                except rospy.ServiceException as e:
                    print(f"Error in inverse kinematics service (left arm): {e}")
                    # Realizar acciones de manejo de errores según sea necesario
            else:
                try:
                    q = calculate_inverse_kinematics_right(x + 0.12, y, z + 0.1, -0.032, -1.525, 0.2)
                    move_right_arm(q[0], q[1], q[2], q[3], q[4], q[5], q[6])
                    move_right_gripper(-0.2)
                    move_right_arm(-0.4, 0, 0, 3, 1, 0, 0)
                except rospy.ServiceException as e:
                    print(f"Error in inverse kinematics service (right arm): {e}")
                    # Realizar acciones de manejo de errores según sea necesario
            print(" >>> Ready!, take the object")
            say("Ready!, take the object")
            rospy.sleep(3)
            current_state = "SM_GO_PLACE"


        # Estado 7
        elif current_state == "SM_GO_PLACE":
            print("ESTADO 7: "+current_state+"  ºººººº")
            print(" >>> I am going to start my way to the destination point")
            say("I am going to start my way to the destination point")
            rospy.sleep(3)
            move_base(-3.0, 0.0, 4)
            go_to_goal_pose(goal_x = requested_location[0], goal_y = requested_location[1])
            if requested_location[0] == 8.0:
            	rospy.sleep(83)
            else:
            	rospy.sleep(52)
            current_state = "SM_ARRIVING"


        # Estado 8
        elif current_state == "SM_ARRIVING":
            print("ESTADO 8: "+current_state+"  ºººººº")
            goal = True
            if goal:
                print(" >>> Ready! reach the goal")
                say("ready! reach the goal")
                rospy.sleep(3)
                print(" >>> I am releasing the object")
                say("I am releasing the object")
                rospy.sleep(5)
                if requested_object == "pringles":
                    move_left_gripper(0.4)
                else:
                    move_right_gripper(0.4)
            current_state = "SM_GIVE_OBJECT"

        # Estado 9
        elif current_state == "SM_GIVE_OBJECT":
            print("ESTADO 9: "+current_state+"  ºººººº")
            print(" >>> I delivered the object")
            say("I delivered the object")
            rospy.sleep(2)
            print(" >>> Mission complete")
            say("Mission complete")
            rospy.sleep(15)
            current_state = "SM_INIT"


        # Estado 10 
        else:
            print("Error in SM. Last state: "+current_state)
            break;
        
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
