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

NAME = "FULL_NAME"

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
    lower_color_limit = [0, 0, 0]
    upper_color_limit = [0, 0, 0]

    # Assign lower and upper color limits according to the requested object
    if obj_name == 'pringles':
        lower_color_limit = np.array([25, 50, 50])
        upper_color_limit = np.array([35, 255, 255])
    else:
        lower_color_limit = np.array([10, 200, 50])
        upper_color_limit = np.array([20, 255, 255])
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower_color_limit, upper_color_limit)
    nonzero_points = cv2.findNonZero(mask)
    if nonzero_points is not None:
        centroid = np.mean(nonzero_points, axis=0, dtype=np.int)
        centroid_x = centroid[0][0]
        centroid_y = centroid[0][1]
        if centroid_x < points.shape[1] and centroid_y < points.shape[0]:
            centroid_cartesian_x = points[centroid_y, centroid_x][0]
            centroid_cartesian_y = points[centroid_y, centroid_x][1]
            centroid_cartesian_z = points[centroid_y, centroid_x][2]

            return [centroid_x, centroid_y, centroid_cartesian_x, centroid_cartesian_y, centroid_cartesian_z]
    
    return [0, 0, 0, 0, 0]  # Si no se encuentra un centroide

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
    
