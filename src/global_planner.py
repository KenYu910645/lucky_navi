#!/usr/bin/env python
# https://zh.wikipedia.org/wiki/A*%E6%90%9C%E5%B0%8B%E6%BC%94%E7%AE%97%E6%B3%95 # Document 
import rospy 
import sys
import time
import operator
import math
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped# Global path
from tf import transformations
# Local module 
from heapdict.heapdict import heapdict # priority queue that can decrease key hd['data'] = priority
from radius_table import radius_table

# These are for testing performance 
import cProfile
import re
import pstats # for sorting result 


#----- Load paramters -----# 
foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]

pub_debug_map = rospy.Publisher('debug_map', OccupancyGrid ,queue_size = 1,  latch=True)

class GLOBAL_PLANNER():
    def __init__(self, footprint):
        #----- Global path -------# 
        self.global_path = Path()
        #----- Current Pose ------# 
        self.current_position = Pose2D()
        #------ Goal ---------#
        self.navi_goal = None # idx
        #------- A* -------# 
        # self.state = "stand_by" # "planning" , "finish" , "unreachable", "timeout"
        self.pq = heapdict() # pq[index] = cost 
        self.came_from = {} # came_from['index'] = index of predesessusor
        self.is_need_pub = False 
        #----- Parameter (Subject to .launch file) ------# 
        self.DEBUG_DRAW_DEBUG_MAP = True 
        self.TIME_ANALYSE = False
        self.EXPLORE_ANIMATE_INTERVEL = 0 # 0 means dont show animation
        self.USE_DIJKSTRA = False
        self.OBSTACLE_FACTOR = 0.1 # bigger

        #------- Debug draw -------# 
        if self.DEBUG_DRAW_DEBUG_MAP:
            self.costs = {}
            self.debug_map = OccupancyGrid()
    
    def reset (self):
        '''
        Clean Current Task , and reset to init.
        '''
        #----- Global path -------#
        self.global_path = Path()
        #----- Current Pose ------# TODO re-get 
        # self.current_position = Pose2D()
        #------ Goal ---------#
        self.navi_goal = None # idx
        #------- A* -------# 
        self.pq = heapdict()
        self.came_from = {}
        self.is_need_pub = False 
        if self.DEBUG_DRAW_DEBUG_MAP:
            self.clean_screen()

    def clean_screen (self):
        self.costs = {}
        self.debug_map.data = [None]*(self.debug_map.info.width * self.debug_map.info.height)
        for i in range((self.debug_map.info.width * self.debug_map.info.height)):
            self.debug_map.data[i] = 0

    
    def initial_pose_goal_CB(self, init_pose):
        rospy.loginfo("Current_position : " + str(init_pose))
        self.current_position.x = init_pose.pose.pose.position.x
        self.current_position.y = init_pose.pose.pose.position.y
        self.current_position.theta = transformations.euler_from_quaternion(self.pose_quaternion_2_list(init_pose.pose.pose.orientation))[2]

    def pose_quaternion_2_list(self, quaternion):
        """
        This function help transfer the geometry_msgs.msg.PoseStameped 
        into (translation, quaternion) <-- lists
        """
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    def plan_do_it(self):
        '''
        Blocking function, return until get a result.
        Return plan result : "finish", "unreachable" , "timeout"
        Return -1 if can't find path
        '''
        global pub_debug_map
        current_pos_idx = XY2idx((self.current_position.x, self.current_position.y))
        #------ First point -------# 
        animate_counter = 0 
        x = current_pos_idx
        self.pq[x] = self.dis_est(x, self.navi_goal)
        while True:
            try:
                #(index, cost)   ,  Get lowest cost in open-set
                (x, x_cost) = self.pq.popitem() 
            except IndexError: # the PQ is empty, means every reachable point has already traversed but still can't find goal.
                return -1  # Can't find path to goal
            
            if x == self.navi_goal: #if GOAL is reached
                rospy.loginfo("[A*] arrive goal !!")
                break
            #iterate neighbor of X 
            for y in self.neighbor(x):
                try: 
                    y_score = self.pq[y]
                except KeyError:# y is not in open set 
                    try:
                        self.came_from[y]
                    except KeyError: # y is a brand new point
                        self.pq[y] = float('inf') # init a new point
                        self.came_from[y] = x 
                        y_score = float('inf')
                    else:  # y is a closed point , igonre it 
                        continue
                # update cost                                   decide distance v.s. cost # self.neighbor_delta_cost(x ,y) * 0  +
                if self.USE_DIJKSTRA: # Dijkstra
                    new_y_score = (x_cost) + neighbor_dist(x,y) + self.neighbor_delta_cost(x ,y)*self.OBSTACLE_FACTOR
                else: # A* 
                    new_y_score = (x_cost - self.dis_est(x, self.navi_goal)) + neighbor_dist(x,y) \
                                + self.dis_est(y, self.navi_goal) + self.neighbor_delta_cost(x ,y)*self.OBSTACLE_FACTOR
                if new_y_score < y_score:# need to update value (decrease key)
                    self.came_from[y] = x            #y is key, x is value//make y become child of X 
                    self.pq[y] = new_y_score
                    if self.DEBUG_DRAW_DEBUG_MAP:
                        self.costs[y] = new_y_score
            #--- Make a animation while exploring -----#
            if self.EXPLORE_ANIMATE_INTERVEL != 0:
                animate_counter += 1
                if animate_counter % self.EXPLORE_ANIMATE_INTERVEL == self.EXPLORE_ANIMATE_INTERVEL - 1:
                    self.generate_debug_costmap()
                    pub_debug_map.publish(self.debug_map)
        #----------------- Publish Path----------------#
        p = self.navi_goal
        dis_sum = 0
        while True: 
            #----- Convert idx -> Pose -------#
            step = PoseStamped()
            step.header.frame_id = "map"
            step.header.stamp = rospy.get_rostime()
            if p == current_pos_idx:
                rospy.loginfo("Finish drawing !!")
                break 
            (step.pose.position.x ,step.pose.position.y) = idx2XY(p)
            self.global_path.poses.append(step)
            # ----get Dis ----# 
            dis_sum += neighbor_dist(p , self.came_from[p])
            #----- Rewine ------# 
            p = self.came_from[p]
        self.global_path.header.frame_id = "map"
        self.global_path.header.stamp = rospy.get_rostime()
        if self.DEBUG_DRAW_DEBUG_MAP:
            self.generate_debug_costmap()
            pub_debug_map.publish(self.debug_map)
        self.is_need_pub = True 
        # ------ Test -----# 
        print ("Total points traversed : " + str(len(self.came_from)))
        print ("distance_GRID = " + str(dis_sum))
        print ("distance_eduli = " + str(self.dis_est(self.navi_goal , current_pos_idx)))
        print ("GRID - EDUELI = " + str(dis_sum - self.dis_est(self.navi_goal , current_pos_idx)))

    def neighbor_delta_cost(self, x ,y):
        if GC.global_costmap.data[y] >= 99: # Solid wall is unpenetrable
            return float("inf")
        else: 
            return GC.global_costmap.data[y] - GC.global_costmap.data[x]
    
    def dis_est(self, n1, n2):
        '''
        estimate the distance between 'n' and 'goal'
        '''
        #### Manhattan
        #return (goal - n) % self.MAP_SIZE + (goal - n) / self.MAP_SIZE

        #### Euclidean
        (x1, y1) = idx2XY(n1)
        (x2, y2) = idx2XY(n2)
        dx = x2 - x1 
        dy = y2 - y1 
        return math.sqrt(dx**2 + dy**2)
        
        #### Chebyshev
        #if (goal - n) % self.MAP_SIZE > (goal - n) / self.MAP_SIZE:
        #    return (goal - n) % self.MAP_SIZE
        #else:
        #    return (goal - n) / self.MAP_SIZE

    def neighbor(self, x):
        '''
        return neighborhood of x, as a List with 4 node.(up, down, right, left,up-right , up-left, down-right, down-left)
        '''
        ans = ( x + GC.width ,      # UP
                x - GC.width ,      # DOWN
                x + 1 ,             # RIGHT
                x - 1 ,             # LEFT
                x + 1 + GC.width ,  # UP-RIGHT
                x - 1 + GC.width ,  # UP-LEFT
                x + 1 - GC.width ,  # DOWN-RIGHT
                x - 1 - GC.width )  # DOWN-LEFT
        #----------------- Boundary check ---------------# 
        ans_wihtout_exceed_boundary = []
        for i in ans:
            if GC.global_costmap.data[i] != -1 and i >= 0 and i < (GC.width-1) * (GC.height-1):# inside map and its not unknow point 
                ans_wihtout_exceed_boundary.append(i)
        return ans_wihtout_exceed_boundary
    def getCurrentPos (self):
        pass 
    def generate_debug_costmap(self):
        '''
        Generate debug_map to show a good looking costmap on rviz.
        Dependency: 
            self.costs
        '''
        # --- get MAX cost of costmap -----# 
        r = (100.0) / self.costs[max(self.costs.items(), key=operator.itemgetter(1))[0]] # r = 100 / max value in costs
        # ---- Norma0lize -------# 
        for i in self.costs:
            self.debug_map.data[i] = int(self.costs[i] * r)


def move_base_simple_goal_CB(navi_goal):
    rospy.loginfo("Target : " + str(navi_goal))
    GP.reset()
    GP.navi_goal = XY2idx((navi_goal.pose.position.x, navi_goal.pose.position.y))
    #------ Check if it's a unreachable goal, e.g. outside map or inside wall )---------# 
    if GC.global_costmap.data[GP.navi_goal] >= 99 or GC.global_costmap.data[GP.navi_goal] == -1:
        rospy.logerr("[A*] Goal is not reachable.")
        return 
    t_start = time.time()
    if GP.TIME_ANALYSE:
        rc = cProfile.run('GP.plan_do_it()','restats')
        p = pstats.Stats('restats')
        p.strip_dirs().sort_stats('name').print_stats()
        p.sort_stats('cumulative').print_stats(30)
        p.sort_stats('time').print_stats(30)
    else: 
        rc = GP.plan_do_it() # This function block until path is found.
    rospy.loginfo("[A*] time spend: " + str(time.time() - t_start))
    if rc == -1 :
        rospy.logerr("[A*] Goal is not reachable.")

def global_map_CB(map):
    t_start = time.time()
    GC.resolution = map.info.resolution
    GC.width = map.info.width
    GC.height = map.info.height
    print ("Resolution: " + str(GC.resolution))
    print ("Width: " + str(GC.width))
    print ("Height: " + str(GC.height))
    GC.global_costmap.header = map.header 
    GC.global_costmap.info = map.info

    # ----- Init Global_costmap -----# 
    t_init_start = time.time()
    GC.global_costmap.data = [None]*(map.info.width * map.info.height)
    for i in range((map.info.width * map.info.height)):
        GC.global_costmap.data[i] = 0
    
    #------ Output : dis2costList ---------# 
    b = pow(0.5, 1 / (GC.circum_rad - GC.inscribe_rad))# Constant
    slope = 99 / (GC.MAX_COST_DISTANCE*GC.resolution - GC.inscribe_rad) # For linear costmap
    for i in range(GC.MAX_COST_DISTANCE):
        if i == 0: # Inside obstacle 
            cost = 100
        elif i*GC.resolution  <= GC.inscribe_rad : # AMR is definitely inside wall -> very danger zone
            cost = 99
        else:  # inflation
            # cost = 99 * pow(b ,(i*self.resolution - self.inscribe_rad)) # exponentail decay
            cost = 99 - (slope*(i*GC.resolution - GC.inscribe_rad)) # For linear costmap
        GC.dis2costList.append(cost)
    # print (dis2costList)
    max_idx = GC.width * GC.height
    print ("INIT OK , takes" + str(time.time() - t_init_start)+ " sec. ")
    
    for ori_map_pixel in range(len(map.data)): # iterate every point in map
        if map.data[ori_map_pixel] == 100: # if points is an obstacle -> create a costmap around it.
            for radius_iter in range(GC.MAX_COST_DISTANCE):# Check pixel surrond it.
                #----- Decide Cost ------#
                cost = GC.dis2costList[radius_iter]
                #----- assign pixel cost -----#
                for relative_pos in radius_table[radius_iter]:
                    relative_idx = ori_map_pixel + relative_pos[0] + relative_pos[1] * GC.width
                    if relative_idx < 0  or relative_idx >= max_idx:  # Avoid data[-1234]
                        pass # Not valid idx 
                    else:
                        if GC.global_costmap.data[relative_idx] < cost:
                            GC.global_costmap.data[relative_idx] = cost
                        else:
                            pass
    
    # TODO maybe this realization lack of efficiency?
    for ori_map_pixel in range(len(map.data)): # iterate every point in map
        if map.data[ori_map_pixel] == -1: # if points is unknow 
            GC.global_costmap.data[ori_map_pixel] = -1 
    print ("Time spend at global_costmap : " + str(time.time() - t_start) + " sec. ")
    GC.is_need_pub = True 

    #---- Debug map for A* -----# 
    if GP.DEBUG_DRAW_DEBUG_MAP:
        GP.debug_map        = OccupancyGrid()
        GP.debug_map.header = GC.global_costmap.header
        GP.debug_map.info   = GC.global_costmap.info
        GP.debug_map.data   = [None]*(GP.debug_map.info.width * GP.debug_map.info.height)
        for i in range((GP.debug_map.info.width * GP.debug_map.info.height)):
            GP.debug_map.data[i] = 0

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


#######################
### Global Function ###
#######################
def idx2XY (idx):
    '''
    idx must be interger
    '''
    reso = GC.resolution
    x = (idx %  GC.width) * reso + GC.global_costmap.info.origin.position.x + reso/2 # Center of point 
    y = (idx // GC.width) * reso + GC.global_costmap.info.origin.position.y + reso/2  # Use // instead of math.floor(), for efficiency
    return (x, y)

def XY2idx(XY_coor):
    '''
    XY_coor = ( x , y)
    '''
    origin = [GC.global_costmap.info.origin.position.x , GC.global_costmap.info.origin.position.y]
    # Y 
    idx =  round((XY_coor[1] - origin[1]) / GC.resolution - 0.5) * GC.width
    # print ("Y : " +  str(idx) )
    # idx =  math.floor((XY_coor[1] - origin[1]) / reso) * width
    # X 
    idx += round((XY_coor[0] - origin[0]) / GC.resolution - 0.5)
    # idx += math.floor((XY_coor[0] - origin[0]) / reso)
    # print ("idx = " + str(idx)) 
    return int(idx)

SQRT_2 = math.sqrt(2)
def neighbor_dist(n1,n2):
    '''
    Euclidean distance 
    return distance between neighbor
    Input: 
    n1   -- neighbor 1 (idx)
    n2   -- neighbor 2 (idx)
    '''
    d_idx = abs(n1-n2)
    if d_idx == 1 or d_idx == GC.width: # Right or Left, Up or Down 
        return GC.resolution
    elif d_idx == GC.width + 1 or d_idx == GC.width -1: # Up-right , up-left, ....
        return GC.resolution*SQRT_2
    else:# There're not neighborhood
        return None 

#----- Declare Class -----# 
GP = GLOBAL_PLANNER(foot_print)
GC = GLOBAL_CARTOGRAPHER(foot_print)

def main(args):

    #----- Init node ------# 
    # global_planner = GLOBAL_PLANNER(foot_print)
    rospy.init_node('global_planner', anonymous=True)
    
    #----- Subscribers ------# 
    rospy.Subscriber('/map', OccupancyGrid, global_map_CB)
    # rospy.Subscriber('/current_position', OccupancyGrid, global_planner.current_position_CB) # Use TF to get it. 
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, move_base_simple_goal_CB)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, GP.initial_pose_goal_CB)
    
    #----- Publisher -------# 
    pub_global_costmap = rospy.Publisher('global_costmap', OccupancyGrid ,queue_size = 10,  latch=True)
    pub_global_path    = rospy.Publisher('global_path', Path ,queue_size = 1,  latch=False)

    #---- Load paramters ------# 
    # GLOBAL paramters loading from rosparam server
    while True :
        try:
            #Get namespace of these parameters
            ns_param = rospy.get_param("/global_planner")
        except:
            rospy.loginfo("docking_navigation parameters are not found in rosparam server, keep on trying...")
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
            continue
        else: # Get paramter from launch file successfully
            GP.OBSTACLE_FACTOR          = ns_param ['obstacle_factor']
            GP.TIME_ANALYSE             = ns_param ['use_time_analyse']
            GP.USE_DIJKSTRA             = ns_param ['use_dijkstra']
            GP.DEBUG_DRAW_DEBUG_MAP     = ns_param ['show_debug_map']
            GP.EXPLORE_ANIMATE_INTERVEL = ns_param ['explore_animate_intervel']
            break  

    r = rospy.Rate(10)#call at 10HZ
    while (not rospy.is_shutdown()):
        if GC.is_need_pub: 
            pub_global_costmap.publish(GC.global_costmap)
            GC.is_need_pub = False
        if GP.is_need_pub:
            pub_global_path.publish(GP.global_path)
            GP.is_need_pub = False
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
        # cProfile.run('main(sys.argv)')
    except rospy.ROSInterruptException:
        pass

    

