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
circum_rad = None 
inscribe_rad = None 

def global_map_CB(map):
    global global_costmap
    print ("Resolution: " + str(map.info.resolution))
    print ("Width: " + str(map.info.width))
    print ("Height: " + str(map.info.height))
    global_costmap.header = map.header 
    global_costmap.info = map.info 
    # print (map.data)
    #-------  Test cartographer ------# 
    
    for ori_map_pixel in range(len(map.data)):
        if map.data[ori_map_pixel] == 100:
            global_costmap.data.append(100)
        elif map.data[ori_map_pixel] == -1:
            global_costmap.data.append(100) # Take unknow eara as obstacle 
        else: #  == 0 
            #------ Get closest obstacle distance from this point .-------# 
            #------  Block output : obstacle_dis -------# 
            obstacle_dis = None 
            for radius_iter in range(20):# Check pixel surrond it.
                if obstacle_dis != None : # Already get the answer
                    break
                for relative_pos in radius_table[radius_iter]: 
                    relative_idx = ori_map_pixel + relative_pos[0] + relative_pos[1] * map.info.width
                    try: 
                        if map.data[relative_idx] == 100 : # is obstacle 
                            obstacle_dis = radius_iter # Get obstacle distance 
                            break
                        else: 
                            pass 
                    except: 
                        pass 
            #---------- append cost pixel to costmap ------------# 
            if obstacle_dis == None : # No obstacle at all 
                global_costmap.data.append(0) # Free space 
            else: 
                # Cost calculate function 
                if obstacle_dis <= inscribe_rad / map.info.resolution: 
                    global_costmap.data.append(99)
                elif obstacle_dis <= circum_rad / map.info.resolution: 
                    global_costmap.data.append(49 + int(50 * (1 - (circum_rad - obstacle_dis*map.info.resolution)  /  (circum_rad - inscribe_rad)  )))
                else: 
                    global_costmap.data.append(int(100 * (1 - obstacle_dis / 20.0)))

        

            
    #global_costmap.data [ int(math.floor(map.info.origin.position.x*-1 / map.info.resolution)  + math.floor( -map.info.origin.position.y / map.info.resolution) * map.info.width) ] = 100
    # global_costmap.data[XY2idx((0,0))] = 100 
    # XY2idx(idx2XY(XY2idx((0.001,-0.001))))

    
    
    '''
    # ------- Test radis_table -------# 
    idx_ori = XY2idx((0.025, 0.025))
    for k in range(40):
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
    '''
    #----- How to get radius table ------# 
    '''
    idx_ori = XY2idx((0.025, 0.025))
    print ("idx_ori" + str(idx_ori))
    total_output = []
    for k in range (40):
        output = list() 
        for i in range(len(map.data)):
            (x,y) = idx2XY(i)
            dx = x - 0.025
            dy = y - 0.025
            if dx*dx + dy*dy <= ((k + 0.01)*map.info.resolution) * ((k + 0.01)*map.info.resolution):
                if global_costmap.data[i] != 50:
                    dIdx = i-idx_ori
                    if dIdx%map.info.width > 1000 :
                        change = dIdx%map.info.width - (map.info.width)
                    else: 
                        change = dIdx%map.info.width
                    output.append([change , int(round(dIdx/float(map.info.width))) ])
                    global_costmap.data[i] = 50
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
    global circum_rad, inscribe_rad
    # Init something
    # v = elevator(1, 8) # 1F ~ 8F 
    rospy.init_node('global_cartographer', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, global_map_CB)
    # for visualization
    pub_global_costmap = rospy.Publisher('global_costmap', OccupancyGrid ,queue_size = 10,  latch=True)
    # ev.init_viz()s
    circum_rad = getCircumscribed(foot_print)
    inscribe_rad = getInscribed(foot_print)

    print ("getCircumscribed: " +  str(circum_rad))
    print ("getInscribed: " +  str(inscribe_rad))

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

