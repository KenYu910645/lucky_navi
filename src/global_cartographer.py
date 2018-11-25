#!/usr/bin/env python
import rospy
import sys
import time
import math
import tf
import tf2_ros
import serial
#from sensor_msgs.msg import Imu, JointState
#from tf.transformations import quaternion_from_euler
#from control_msgs.msg import JointControllerState
#from geometry_msgs.msg import Twist, TransformStamped
#from geometry_msgs.msg import PoseStamped, TransformStamped, Point, PointStamped
from nav_msgs.msg import OccupancyGrid
#from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray

# Foot print (0,0) must be the center of incribed circle 
foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]
global_costmap = OccupancyGrid()

def global_map_CB(map):
    global global_costmap
    print ("Resolution: " + str(map.info.resolution))
    print ("Width: " + str(map.info.width))
    print ("Height: " + str(map.info.height))
    global_costmap = map
    # print (map.data)
    #----- Test ------# 
    # for i in map.data: 
    
    global_costmap.data [ math.floor(map.info.origin.position.x*-1 / map.info.resolution)  + math.floor( -map.info.origin.position.y / map.info.resolution) * map.info.width ] = 100
        
    '''
    for i in map.data:
        if i == 0 : 
            global_costmap.data.append(0)
        elif i == 100: 
            global_costmap.data.append(100)
        elif i == -1: 
            global_costmap.data.append(-1)
    '''
def getInscribed(footprint):
    shortest_dis = float('inf')
    for i in range(len(footprint)): # i and i-1 
        x2 = footprint[i][0]
        y2 = footprint[i][1]
        x1 = footprint[i-1][0]
        y1 = footprint[i-1][1]

        try: 
            slope  = (y2-y1) / (x2-x1)
        except:
            # print ("[getInscribed] Error when calculate Slope, divived by zero?")
            slope = (y2-y1) / 0.00001
        
        dis = abs(y1 - slope * x1) / math.sqrt(slope*slope + 1)
        if shortest_dis > dis :
            shortest_dis = dis # update distance
    return shortest_dis
def getCircumscribed (footprint):
    shortest_dis = float('inf')
    for i in footprint:
        dis = math.sqrt( i[0]*i[0] + i[1]*i[1] )
        if shortest_dis > dis :
            shortest_dis = dis # update distance
    return shortest_dis

    

def main(args):
    # Init something
    # v = elevator(1, 8) # 1F ~ 8F 
    rospy.init_node('global_cartographer', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, global_map_CB)
    # for visualization
    pub_global_costmap = rospy.Publisher('global_costmap', OccupancyGrid ,queue_size = 10,  latch=True)
    # ev.init_viz()s
    
    print ("getCircumscribed: " +  str(getCircumscribed(foot_print)))
    print ("getInscribed: " +  str(getInscribed(foot_print)))

    #call at 10HZ
    r = rospy.Rate(1)    
    while (not rospy.is_shutdown()):
        # Draw out button 
        # pub_markers.publish(markerArray)
        pub_global_costmap.publish(global_costmap)
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

