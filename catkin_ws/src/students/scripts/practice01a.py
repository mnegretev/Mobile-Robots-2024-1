#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# PRACTICE 01 - MAPS 
#
# Instructions:
# Complete the code necessary to inflate the obstacles given an occupancy grid map and
# a number of cells to inflate. 
#

import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "Erik Yoel Santana Apreza"

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    for i in range(0, height):
       for j in range(0, width):
           if static_map[i,j]>50:
               for k1 in range(-inflation_cells, inflation_cells+1):
                    for k2 in range(-inflation_cells, inflation_cells+1):
                           inflated[i+k1, j+k2] =100
        
    return inflated

def callback_inflated_map(req):
    global inflated_map
    return GetMapResponse(map=inflated_map)

def main():
    global cost_map, inflated_map
    print("PRACTICE 01 A - Map Inflation - " + NAME)
    rospy.init_node("practice01a")
    rospy.wait_for_service('/static_map')
    pub_map  = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    grid_map = rospy.ServiceProxy("/static_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    rospy.Service('/inflated_map', GetMap, callback_inflated_map)
    loop = rospy.Rate(1)
    
    inflation_radius = 0.15
    if rospy.has_param("~inflation_radius"):
        inflation_radius = rospy.get_param("~inflation_radius")
    while not rospy.is_shutdown():
        inflated_map_data = get_inflated_map(grid_map, round(inflation_radius/res))
        inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (width*height, 1)))
        inflated_map      = OccupancyGrid(info=map_info, data=inflated_map_data)
        pub_map.publish(callback_inflated_map(GetMapRequest()).map)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
