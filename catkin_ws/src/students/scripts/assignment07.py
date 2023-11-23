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

NAME = "Adán Yareth Guevara Mendoza"

def segment_by_color(img_bgr, points, obj_name):
	global bin_img
    #
    # TODO:
    # - Assign lower and upper color limits according to the requested object:
    #   If obj_name == 'pringles': [25, 50, 50] - [35, 255, 255]
    #   otherwise                : [10,200, 50] - [20, 255, 255]
    
    

	if obj_name == "pringles":
		hsv_inferior = numpy.array([25, 50, 50])
		hsv_superior = numpy.array([35, 255, 255])
	else:
		hsv_inferior = numpy.array([10, 200, 50])
		hsv_superior = numpy.array([20, 255, 255])


    
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
	hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    
    # - Determine the pixels whose color is in the selected color range.
    #   Check online documentation for cv2.inRange
	bin_img = cv2.inRange(hsv_img, hsv_inferior, hsv_superior)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
	bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel)
    
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    
	nonzero_elements = cv2.findNonZero(bin_img)
	cv2.bitwise_and(img_bgr, img_bgr, mask=nonzero_elements)
	centroid_px = cv2.mean(nonzero_elements)
	px, py = int(centroid_px[0]), int(centroid_px[1])
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    #
    
	x = points[py, px][0]
	y = points[py, px][1]
	z = points[py, px][2]
	return [px,py,x,y,z]

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
    bin_img = numpy.zeros((480, 640, 3), numpy.uint8)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        #cv2.imshow("Color Segmentation", img_bgr)
        #cv2.waitKey(1)
        loop.sleep()
    
if __name__ == '__main__':
    main()
    
