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

NAME = "Guillen Castillo Jorge Luis"

# Límites de color para "pringles"
H_MIN_PRINGLES, S_MIN_PRINGLES, V_MIN_PRINGLES = 25, 50, 50
H_MAX_PRINGLES, S_MAX_PRINGLES, V_MAX_PRINGLES = 35, 255, 255

# Límites de color para "drink"
H_MIN_DRINK, S_MIN_DRINK, V_MIN_DRINK = 10, 200, 50
H_MAX_DRINK, S_MAX_DRINK, V_MAX_DRINK = 20, 255, 255

def segment_by_color(img_bgr, points, obj_name):
    # Límites de color según el objeto solicitado
    if obj_name == 'pringles':
        lower_color_limit = numpy.array([H_MIN_PRINGLES, S_MIN_PRINGLES, V_MIN_PRINGLES])
        upper_color_limit = numpy.array([H_MAX_PRINGLES, S_MAX_PRINGLES, V_MAX_PRINGLES])
    elif obj_name == 'drink':
        lower_color_limit = numpy.array([H_MIN_DRINK, S_MIN_DRINK, V_MIN_DRINK])
        upper_color_limit = numpy.array([H_MAX_DRINK, S_MAX_DRINK, V_MAX_DRINK])
    else:
        # Límites de color predeterminados
        lower_color_limit = numpy.array([20, 100, 100])
        upper_color_limit = numpy.array([30, 255, 255])

    # Transformar la imagen del espacio BGR al espacio HSV
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Determinar los píxeles de la imagen que pertenecen al rango de color elegido
    mask = cv2.inRange(img_hsv, lower_color_limit, upper_color_limit)

    # Encontrar los índices de los píxeles que pertenecen al rango de color
    nonzero = cv2.findNonZero(mask)

    if nonzero is not None:
        # Utilizando los índices y la nube de puntos, determinar el centroide del objeto
        mean_val = cv2.mean(nonzero)
        r, c = int(mean_val[1]), int(mean_val[0])
        x, y, z = points[r, c]

        return [r, c, x, y, z]

    return [0, 0, 0, 0, 0]

def callback_find_object(req):
    global pub_point, img_bgr
    print("Trying to find object: " + req.name)
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.point_cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r, g, b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((numpy.asarray(b, dtype='uint8'), numpy.asarray(g, dtype='uint8'), numpy.asarray(r, dtype='uint8')))
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
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        loop.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

    
