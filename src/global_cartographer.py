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
from radius_table import radius_table
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
    global_costmap.header = map.header 
    global_costmap.info = map.info 
    # print (map.data)
    #----- Test ------# 
    
    for i in map.data:
        if i == 0 : 
            global_costmap.data.append(0)
        elif i == 100: 
            global_costmap.data.append(100)
        elif i == -1: 
            global_costmap.data.append(-1)
    #global_costmap.data [ int(math.floor(map.info.origin.position.x*-1 / map.info.resolution)  + math.floor( -map.info.origin.position.y / map.info.resolution) * map.info.width) ] = 100
    # global_costmap.data[XY2idx((0,0))] = 100 
    # XY2idx(idx2XY(XY2idx((0.001,-0.001))))

    #---- Test on radius table -----# 
    
    idx_ori = XY2idx((0.025, 0.025))
    for k in range(30):
        for i in radius_table[k-1]:
            idx =idx_ori
            idx += i[0]
            idx += i[1] * map.info.width
            global_costmap.data[idx] = 0
        for i in radius_table[k]:
            idx =idx_ori
            idx += i[0]
            idx += i[1] * map.info.width
            global_costmap.data[idx] = 50
        time.sleep(1) # For watch slowly 
    
    #----- How to get radius table ------# 
    '''
    idx_ori = XY2idx((0.025, 0.025))
    print ("idx_ori" + str(idx_ori))
    total_output = []
    for k in range (40):
        # print ("=====================================================")
        # print ( + " Layer : ")
        output = list() 
        for i in range(len(map.data)):
            (x,y) = idx2XY(i)
            dx = x - 0.025
            dy = y - 0.025
            if dx*dx + dy*dy <= ((k + 0.01)*map.info.resolution) * ((k + 0.01)*map.info.resolution):
                if global_costmap.data[i] != 50:
                    dIdx = i-idx_ori
                    if dIdx%map.info.width > 1000 :
                        change = dIdx%map.info.width - map.info.width
                    else: 
                        change = dIdx%map.info.width
                    output.append([change , int(math.floor(dIdx/map.info.width)) ])
                    global_costmap.data[i] = 50
        
        #print (str(k) +" = " + str(output ))
        total_output.append(output)
    print ("radius_table = " + str(total_output))
    '''
        
def idx2XY (idx):
    '''
    idx must be interger
    '''
    reso  = global_costmap.info.resolution
    width = global_costmap.info.width
    height = global_costmap.info.height
    origin = [global_costmap.info.origin.position.x , global_costmap.info.origin.position.y]

    x = (idx % width) * reso + origin[0] + reso/2 # Center of point 
    # y = round(idx / width) * reso + origin[1] + reso/2 
    y = math.floor(idx / width) * reso + origin[1] + reso/2 

    # print ("(x ,y ) = " + str((x,y)))
    return (x, y)

def XY2idx( XY_coor ):
    '''
    XY_coor = ( x , y)
    '''
    reso  = global_costmap.info.resolution
    width = global_costmap.info.width
    height = global_costmap.info.height
    origin = [global_costmap.info.origin.position.x , global_costmap.info.origin.position.y]

    # Y 
    idx =  round((XY_coor[1] - origin[1]) / reso - 0.5) * width
    # print ("Y : " +  str(idx) )
    # idx =  math.floor((XY_coor[1] - origin[1]) / reso) * width
    # X 
    idx += round((XY_coor[0] - origin[0]) / reso - 0.5)
    # idx += math.floor((XY_coor[0] - origin[0]) / reso)


    # print ("idx = " + str(idx)) 
    return int(idx) 



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

