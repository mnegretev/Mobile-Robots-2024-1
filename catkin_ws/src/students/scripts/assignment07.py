#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# ASSIGNMENT 07 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of an object 
# given a colored point cloud using color segmentation.
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.srv import RecognizeObject, RecognizeObjectResponse
from vision_msgs.msg import VisionObject

NAME = "JARQUIN LOPEZ DANIEL EDUARDO"

def segment_by_color(img_bgr, points, obj_name):
    if obj_name == 'pringles':
        # Color range for 'pringles' object in HSV
        lower_color = numpy.array([25, 50, 50])
        upper_color = numpy.array([35, 255, 255])
    else:
        # Color range for other objects in HSV
        lower_color = numpy.array([10, 200, 50])
        upper_color = numpy.array([20, 255, 255])

    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, lower_color, upper_color)

    nonzero_points = cv2.findNonZero(mask)
    if nonzero_points is not None:
        centroid = cv2.mean(nonzero_points)
        r, c = centroid[0], centroid[1]
    else:
        return [0, 0, 0, 0, 0]

    x = points[int(r), int(c)][0]
    y = points[int(r), int(c)][1]
    z = points[int(r), int(c)][2]

    return [r, c, x, y, z]

def callback_find_object(req):
    global pub_point, img_bgr
    print("Trying to find object: " + req.name)
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.point_cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    [r, c, x, y, z] = segment_by_color(img_bgr, arr, req.name)
    hdr = Header(frame_id='realsense_link', stamp=rospy.Time.now())
    pub_point.publish(PointStamped(header=hdr, point=Point(x=x, y=y, z=z)))
    cv2.circle(img_bgr, (int(r), int(c)), 20, [0, 255, 0], thickness=3)
    resp = RecognizeObjectResponse()
    resp.recog_object.pose.position.x = x
    resp.recog_object.pose.position.y = y
    resp.recog_object.pose.position.z = z
    return resp

def main():
    global pub_point, img_bgr
    print("ASSIGNMENT 07 - " + NAME)
    rospy.init_node("color_segmentation")
    rospy.Service("/vision/obj_reco/recognize_object", RecognizeObject, callback_find_object)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    img_bgr = numpy.zeros((480, 640, 3), numpy.uint8)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.imshow("Color Segmentation", img_bgr)
        cv2.waitKey(1)
        loop.sleep()
    
if __name__ == '__main__':
    main()
    
