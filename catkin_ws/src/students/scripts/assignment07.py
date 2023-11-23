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

NAME = "Arriaga Mejia Jose Carlos"

def segment_by_color(img_bgr, points, obj_name):
    #
    # TODO:
    # - Assign lower and upper color limits according to the requested object:
    #   If obj_name == 'pringles': [25, 50, 50] - [35, 255, 255]
    #   otherwise                : [10,200, 50] - [20, 255, 255]
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the selected color range.
    #   Check online documentation for cv2.inRange
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    #
    if (obj_name == 'pringles'): 
        min_valor = numpy.array([25, 50, 50]) 
        max_valor = numpy.array([35, 255, 255])
    else: #[10,200, 50] - [20, 255, 255]
        min_valor = numpy.array([10,200, 50]) 
        max_valor = numpy.array([20, 255, 255])
    img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
    img_bin = cv2.inRange(img_hsv,min_valor, max_valor)
    img_non_zero = cv2.findNonZero(img_bin)
    centroide_pixel = cv2.mean(img_non_zero)
    centroide_r, centroide_c = centroide_pixel[0], centroide_pixel[1]
    x, y, z = 0,0,0
    for cord in img_non_zero:
        c,r = cord[0]           # columna , renglon
        x += points[r,c][0] # accedemos a la posicion actual (r,c) en la imagen y al valor del 1r elemento (coordenada x)
        y += points[r,c][1]
        z += points[r,c][2]
    centroide_x = x/len(img_non_zero)
    centroide_y = y/len(img_non_zero)
    centroide_z = z/len(img_non_zero)
    return[centroide_r,centroide_c, centroide_x, centroide_y, centroide_z]

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
    
