#!/usr/bin/env python
# https://zh.wikipedia.org/wiki/A*%E6%90%9C%E5%B0%8B%E6%BC%94%E7%AE%97%E6%B3%95 # Document 
import time
import math
from heapdict.heapdict import heapdict # priority queue that can decrease key hd['data'] = priority
from nav_msgs.msg import OccupancyGrid # Global map 
from geometry_msgs.msg import PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped   # Global path 
from radius_table import radius_table
from global_cartographer import GLOBAL_CARTOGRAPHER
import rospy 
import sys 
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
from tf import transformations

# These are for testing performance 
import cProfile
import re
import pstats # for sorting result 

#----- Flags ------# 
DEBUG_DRAW_A_START_SET = False 
TIME_ANALYSE = False 

#----- Load paramters -----# 
foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]

GC = GLOBAL_CARTOGRAPHER(foot_print)
pub_marker = rospy.Publisher('markers', MarkerArray,queue_size = 1,  latch=False )

class GLOBAL_PLANNER():
    def __init__(self, footprint):
        #----- Global path -------# 
        self.global_path = PoseArray()
        #----- Current Pose ------# 
        self.current_position = Pose2D()
        #------ Goal ---------#
        self.navi_goal = None # idx
        #------- A* -------# 
        # self.state = "stand_by" # "planning" , "finish" , "unreachable", "timeout"
        self.pq = heapdict() # pq[index] = cost 
        self.came_from = {} # came_from['index'] = index of predesessusor
        # self.is_reachable = True  # TODO state????
        self.is_need_pub = False 
        #------- Debug draw -------# 
        self.markerArray = MarkerArray()
    def reset (self):
        '''
        Clean Current Task , and reset to init.
        '''
        #----- Global path -------#
        self.global_path = PoseArray()
        #----- Current Pose ------# TODO re-get 
        # self.current_position = Pose2D()
        #------ Goal ---------#
        self.navi_goal = None # idx
        #------- A* -------# 
        self.pq = heapdict()
        self.came_from = {}
        self.is_need_pub = False 
        self.clean_screen()

    def clean_screen (self):
        #------- clean screen -------#
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.action = marker.DELETEALL
        self.markerArray.markers.append(marker)
        pub_marker.publish(self.markerArray)

        #------- Debug draw -------# 
        self.markerArray = MarkerArray()
    
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
        global pub_marker
        current_pos_idx = self.XY2idx((self.current_position.x, self.current_position.y))
        #------ First point -------# 
        x = current_pos_idx
        self.pq[x] = self.neighbor_dist(x,x) + self.goal_dis_est(x, self.navi_goal)
        while True:
            try:
                #(index, cost)   ,  Get lowest cost in open-set
                (x, x_cost) = self.pq.popitem() 
            except IndexError: # the PQ is empty, means every reachable point has already traversed but still can't find goal.
                return -1  # Can't find path to goal
            # print ("Current node : "+ str(x)) 
            
            if x == self.navi_goal: #if GOAL is reached
                rospy.loginfo("[A*] arrive goal !!")
                break
            
            # Debug
            if DEBUG_DRAW_A_START_SET:
                self.set_point(x, 210, 188 , 167)
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
                        if DEBUG_DRAW_A_START_SET:
                            self.set_point(y, 255, 255 , 0)
                    else:  # y is a closed point , igonre it 
                        continue
                # update cost                                   decide distance v.s. cost
                new_y_score = x_cost + self.neighbor_dist(x,y) + self.neighbor_delta_cost(x ,y) * 1  + self.goal_dis_est(y, self.navi_goal)*0.1 # 0.1  # Cost: y -x (0 ~ 100)
                if new_y_score < y_score:# need to update value (decrease key)
                    self.came_from[y] = x            #y is key, x is value//make y become child of X 
                    self.pq[y] = new_y_score
            #if DEBUG_DRAW_A_START_SET:
            #   pub_marker.publish(self.markerArray)
        #----------------- Publish Path----------------#
        p = self.navi_goal
        dis_sum = 0
        while True: 
            #----- Convert idx -> Pose -------#
            step = Pose()
            if p == current_pos_idx:
                rospy.loginfo("Finish drawing !!")
                break 
            (step.position.x ,step.position.y) = self.idx2XY(p)
            self.set_point(p, 255, 255, 255 )
            # ----get Dis ----# 
            dis_sum += self.neighbor_dist(p , self.came_from[p])
            #----- Rewine ------# 
            p = self.came_from[p]
        pub_marker.publish(self.markerArray)
        self.is_need_pub = True 
        # ------ Test -----# 
        print ("Total points traversed : " + str(len(self.came_from)))
        print ("distance_GRID = " + str(dis_sum))
        print ("distance_eduli = " + str(self.neighbor_dist(self.navi_goal , current_pos_idx)))
        print ("GRID - EDUELI = " + str(dis_sum - self.neighbor_dist(self.navi_goal , current_pos_idx)))

    def neighbor_delta_cost(self, x ,y):
        #
        # rospy.loginfo("y : " + str(y))
        if GC.global_costmap.data[y] >= 99:
            return float("inf")
        else: 
            return GC.global_costmap.data[y] - GC.global_costmap.data[x]
    
    def neighbor_dist(self,n1,n2):
        '''
        Euclidean distance 
        return distance between neighbor
        Input: 
        n1   -- neighbor 1
        n2   -- neighbor 2
        '''
        (x1, y1) = self.idx2XY(n1)
        (x2, y2) = self.idx2XY(n2)
        dx = x2 - x1 
        dy = y2 - y1 

        return math.sqrt(dx**2 + dy**2)
    
    def goal_dis_est(self, n, goal):
        '''
        estimate the distance between 'n' and 'goal'
        '''
        #### Manhattan
        #return (goal - n) % self.MAP_SIZE + (goal - n) / self.MAP_SIZE

        #### Euclidean
        return self.neighbor_dist(n,self.navi_goal)
        
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
    
    ############################### SAME AS GLOBAL_CARTOPGRPHER ##################################
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

    ############################### SAME AS GLOBAL_CARTOPGRPHER #################################

    def set_point(self, idx ,r ,g ,b ):
        '''
        Set Point at MarkArray 
        '''
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = idx 
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = r/255.0
        marker.color.g = g/255.0
        marker.color.b = b/255.0
        marker.pose.orientation.w = 1.0
        (marker.pose.position.x , marker.pose.position.y) = self.idx2XY(idx)
        self.markerArray.markers.append(marker)

    def getCurrentPos (self):
        pass 
#----- Declare Class -----# 
GP = GLOBAL_PLANNER(foot_print)

def move_base_simple_goal_CB(navi_goal):
    rospy.loginfo("Target : " + str(navi_goal))
    GP.reset()
    GP.navi_goal = GP.XY2idx((navi_goal.pose.position.x, navi_goal.pose.position.y))
    #------ Check if it's a unreachable goal, e.g. outside map or inside wall )---------# 
    if GC.global_costmap.data[GP.navi_goal] >= 99 or GC.global_costmap.data[GP.navi_goal] == -1:
        rospy.logerr("[A*] Goal is not reachable.")
        return 
    t_start = time.time()
    if TIME_ANALYSE:
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

def main(args):

    #----- Init node ------# 
    global_planner = GLOBAL_PLANNER(foot_print)
    rospy.init_node('global_planner', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, GC.global_map_CB)
    # rospy.Subscriber('/current_position', OccupancyGrid, global_planner.current_position_CB) # Use TF to get it. 
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, move_base_simple_goal_CB)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, global_planner.initial_pose_goal_CB)
    
    pub_global_costmap = rospy.Publisher('global_costmap', OccupancyGrid ,queue_size = 10,  latch=True)
    pub_global_path = rospy.Publisher('global_path', PoseArray ,queue_size = 10,  latch=False)
    
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

    

