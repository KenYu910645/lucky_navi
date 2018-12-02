#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid # Global map 
from geometry_msgs import PoseArray # Global path 
from radius_table import radius_table
import rospy 
import sys 

class GLOBAL_PLANNER():
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
        self.global_costmap = OccupancyGrid()
        #----- Global path -------# 
        self.global_path = PoseArray()
    
    def current_position_CB(self, current_position):
        self.current_position = current_position
    
    def navi_goal_CB(self, navi_goal):
        self.navi_goal = navi_goal
    
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
        for i in range(self.MAX_COST_DISTANCE):
            if i == 0: 
                cost = 100
            elif i*self.resolution  <= self.inscribe_rad : 
                cost = 99
            else: 
                cost = 99 * pow(b ,(i*self.resolution - self.inscribe_rad))
            self.dis2costList.append(cost)
        # print (dis2costList)
        max_idx = self.width * self.height
        print ("INIT OK , takes" + str(time.time() - t_init_start)+ " sec. ")
        
        for ori_map_pixel in range(len(map.data)):
            if map.data[ori_map_pixel] == 100:
                for radius_iter in range(self.MAX_COST_DISTANCE):# Check pixel surrond it.
                    #----- Decide Cost ------#
                    cost = self.dis2costList[radius_iter]
                    #----- assign pixel cost -----# 
                    for relative_pos in radius_table[radius_iter]:
                        relative_idx = ori_map_pixel + relative_pos[0] + relative_pos[1] * self.width
                        if relative_idx < 0  or relative_idx >= max_idx:  # Avoid data[-1234]
                            pass# Not valid idx 
                        else:
                            if self.global_costmap.data[relative_idx] < cost:
                                self.global_costmap.data[relative_idx] = cost
                            else:
                                pass
        print ("Time spend at global_costmap : " + str(time.time() - t_start) + " sec. ")
        self.is_need_pub = True 

    def idx2XY (self, idx):
        '''
        idx must be interger
        '''
        origin = [self.global_costmap.info.origin.position.x , self.global_costmap.info.origin.position.y]

        x = (idx % self.width) * self.resolution + origin[0] + self.resolution/2 # Center of point 
        # y = round(idx / width) * reso + origin[1] + reso/2 
        y = math.floor(idx / self.width) * self.resolution + origin[1] + self.resolution/2 

        # print ("(x ,y ) = " + str((x,y)))
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


def main(args):
    # Init something
    # v = elevator(1, 8) # 1F ~ 8F 
    
    #----- Load paramters -----# 
    foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]
    
    #----- Init node ------# 
    global_planner = GLOBAL_PLANNER(foot_print)
    rospy.init_node('global_planner', anonymous=True)
    rospy.Subscriber('/global_costmap', OccupancyGrid, global_planner.global_map_CB)
    rospy.Subscriber('/current_position', OccupancyGrid, global_planner.current_position_CB)
    rospy.Subscriber('/navi_goal', OccupancyGrid, global_planner.navi_goal_CB)

    pub_global_path = rospy.Publisher('global_costmap', PoseArray ,queue_size = 10,  latch=True)
    
    r = rospy.Rate(10)#call at 10HZ
    while (not rospy.is_shutdown()):
        if global_planner.is_need_pub: 
            pub_global_path.publish(global_planner.global_path)
            global_planner.is_need_pub = False 
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass