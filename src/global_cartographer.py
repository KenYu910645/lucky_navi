#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid
from radius_table import radius_table
import rospy 
import sys 

class GLOBAL_CARTOGRAPHER():
    def __init__(self, footprint): 
        self.footprint = footprint
        self.circum_rad = self.getCircumscribed(footprint)
        self.inscribe_rad = self.getInscribed(footprint)
        self.dis2costList = []
        self.MAX_COST_DISTANCE = 20  # 17 pixel radius 
        self.is_need_pub = False # == True, if Global map is need pub.
        #---- Map info -----# 
        self.resolution = None 
        self.width = None 
        self.height = None 
        #----- Global costmap ------# 
        self.global_costmap = OccupancyGrid() # Output

    def global_map_CB(self, map):
        t_start = time.time()
        self.resolution = map.info.resolution
        self.width = map.info.width
        self.height = map.info.height
        print ("Resolution: " + str(self.resolution))
        print ("Width: " + str(self.width))
        print ("Height: " + str(self.height))
        self.global_costmap.header = map.header 
        self.global_costmap.info = map.info

        # ----- Init Global_costmap -----# 
        t_init_start = time.time()
        self.global_costmap.data = [None]*(map.info.width * map.info.height)
        for i in range((map.info.width * map.info.height)):
            self.global_costmap.data[i] = 0
        
        #------ Output : dis2costList ---------# 
        b = pow(0.5, 1 / (self.circum_rad - self.inscribe_rad))# Constant
        slope = 99 / (self.MAX_COST_DISTANCE*self.resolution - self.inscribe_rad) # For linear costmap
        for i in range(self.MAX_COST_DISTANCE):
            if i == 0: # Inside obstacle 
                cost = 100
            elif i*self.resolution  <= self.inscribe_rad : # AMR is definitely inside wall -> very danger zone
                cost = 99
            else:  # inflation
                # cost = 99 * pow(b ,(i*self.resolution - self.inscribe_rad)) # exponentail decay
                cost = 99 - (slope*(i*self.resolution - self.inscribe_rad)) # For linear costmap
            self.dis2costList.append(cost)
        # print (dis2costList)
        max_idx = self.width * self.height
        print ("INIT OK , takes" + str(time.time() - t_init_start)+ " sec. ")
        
        for ori_map_pixel in range(len(map.data)): # iterate every point in map
            if map.data[ori_map_pixel] == 100: # if points is an obstacle -> create a costmap around it.
                for radius_iter in range(self.MAX_COST_DISTANCE):# Check pixel surrond it.
                    #----- Decide Cost ------#
                    cost = self.dis2costList[radius_iter]
                    #----- assign pixel cost -----#
                    for relative_pos in radius_table[radius_iter]:
                        relative_idx = ori_map_pixel + relative_pos[0] + relative_pos[1] * self.width
                        if relative_idx < 0  or relative_idx >= max_idx:  # Avoid data[-1234]
                            pass # Not valid idx 
                        else:
                            if self.global_costmap.data[relative_idx] < cost:
                                self.global_costmap.data[relative_idx] = cost
                            else:
                                pass
        
        # TODO maybe this realization lack of efficiency?
        for ori_map_pixel in range(len(map.data)): # iterate every point in map
            if map.data[ori_map_pixel] == -1: # if points is unknow 
                self.global_costmap.data[ori_map_pixel] = -1 


        print ("Time spend at global_costmap : " + str(time.time() - t_start) + " sec. ")
        self.is_need_pub = True 
        
        '''
        Iterate all pixel at map  : TOO slow, takes about 20 sec to initial 
        b = pow(0.5, 1 / (circum_rad - inscribe_rad))# Constant 

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
                for radius_iter in range(17):# Check pixel surrond it.
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
                    if obstacle_dis*map.info.resolution  <= inscribe_rad : 
                        global_costmap.data.append(99)
                    else: 
                        cost = 99 * pow(b ,(obstacle_dis*map.info.resolution - inscribe_rad))
                        global_costmap.data.append (cost) 
        print ("Time spend at global_costmap : " + str(time.time() - t_start) + " sec. ")
        #global_costmap.data [ int(math.floor(map.info.origin.position.x*-1 / map.info.resolution)  + math.floor( -map.info.origin.position.y / map.info.resolution) * map.info.width) ] = 100
        # global_costmap.data[XY2idx((0,0))] = 100 
        # XY2idx(idx2XY(XY2idx((0.001,-0.001))))
        '''
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

    def idx2XY (self, idx):
        '''
        idx must be interger
        '''
        reso = GC.resolution
        x = (idx %  GC.width) * reso + GC.global_costmap.info.origin.position.x + reso/2 # Center of point 
        y = (idx // GC.width) * reso + GC.global_costmap.info.origin.position.y + reso/2  # Use // instead of math.floor(), for efficiency
        return (x, y)

    def XY2idx(self,  XY_coor ):
        '''
        XY_coor = ( x , y)
        '''
        origin = [self.global_costmap.info.origin.position.x , self.global_costmap.info.origin.position.y]
        # Y 
        idx =  round((XY_coor[1] - origin[1]) / self.resolution - 0.5) * self.width
        # print ("Y : " +  str(idx) )
        # idx =  math.floor((XY_coor[1] - origin[1]) / reso) * width
        # X 
        idx += round((XY_coor[0] - origin[0]) / self.resolution - 0.5)
        # idx += math.floor((XY_coor[0] - origin[0]) / reso)
        # print ("idx = " + str(idx)) 
        return int(idx)

    def getInscribed(self, footprint):
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

    def getCircumscribed (self, footprint):
        shortest_dis = float('inf')
        for i in footprint:
            dis = math.sqrt( i[0]*i[0] + i[1]*i[1] )
            if shortest_dis > dis :
                shortest_dis = dis # update distance
        return shortest_dis

