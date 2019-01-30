#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid # Global map 
from geometry_msgs.msg import PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped   # Global path 
from radius_table import radius_table
from global_cartographer import GLOBAL_CARTOGRAPHER
import rospy 
import sys 
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
from tf import transformations

#----- Flags ------# 
DEBUG_DRAW_A_START_SET = False  

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
        self.openset = [] 
        self.closedset = []
        self.g_score = {}
        # self.h_score = {}
        # self.f_score = {} # F_score = h_score + g_score
        self.came_from = {}
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
        self.openset = {}  # -> dict() , combine with f_score[] 
        self.closedset = {} # -> dict()
        self.g_score = {}
        # self.h_score = {}
        # self.f_score = {} # F_score = h_score + g_score
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
    
    def move_base_simple_goal_CB(self, navi_goal):
        rospy.loginfo("Target : " + str(navi_goal))
        self.reset()
        self.navi_goal = self.XY2idx((navi_goal.pose.position.x, navi_goal.pose.position.y))
        t_start = time.time()
        self.plan_do_it()
        rospy.loginfo("[A*] time spend: " + str(time.time() - t_start))

    def pose_quaternion_2_list(self, quaternion):
        """
        This function help transfer the geometry_msgs.msg.PoseStameped 
        into (translation, quaternion) <-- lists
        """
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    def plan_do_it(self):
        '''
        Time Loop
        Return plan result : "finish", "unreachable" , "timeout"
        '''
        global pub_marker
        current_pos_idx = self.XY2idx((self.current_position.x, self.current_position.y))
        x = current_pos_idx
        # self.openset.append(x)
        self.g_score[x] = self.neighbor_dist(x,x)
        # self.h_score[x] = self.goal_dis_est(x, self.navi_goal)
        self.openset[x] = self.g_score[x] + self.goal_dis_est(x, self.navi_goal)

        is_finish_plan = False 
        while not is_finish_plan : 
            #if len(self.openset) == 0:
            #    # Can't find a way to Goal 
            #    self.state = "unreachable"
            #    #self.is_reachable = False
            #    return
            x = self.lowest()#  x -  having the lowest f_score[] value in openset
            # print ("Current node : "+ str(x)) 
            
            if x == self.navi_goal: #if GOAL is reached
                rospy.loginfo("[A*] arrive goal !!")
                is_finish_plan = True 
                break
            elif x == -1 : 
                rospy.loginfo("[A*] Can't find lowest set.")
                continue
            
            # Debug
            if DEBUG_DRAW_A_START_SET:
                self.set_point(x, 210, 188 , 167)
            x_count = 0
            #Find neighbor of X 
            for y in self.neighbor(x):
                if y in self.closedset: # if y is already closed
                    if self.closedset[y] < 8:   
                        self.closedset[y] += 1 # X is going to be closedset
                    elif self.closedset[y] >= 8: # Can get rid of y  
                        del self.closedset[y]
                    x_count += 1
                else: 
                    tentative_g_score = self.g_score[x] + self.neighbor_dist(x,y) + self.neighbor_delta_cost(x ,y) * 0.1  # Cost: y -x (0 ~ 100)
                    try: 
                        if tentative_g_score < self.g_score[y]:
                            self.came_from[y] = x            #y is key, x is value//make y become child of X 
                            self.g_score[y] = tentative_g_score
                            self.openset[y] = self.g_score[y] + self.goal_dis_est(y, self.navi_goal)
                    except : # y is not in openset
                        self.came_from[y] = x            #y is key, x is value//make y become child of X 
                        self.g_score[y] = tentative_g_score
                        self.openset[y] = self.g_score[y] + self.goal_dis_est(y, self.navi_goal)
                        #---------- Debug ---------#
                        if DEBUG_DRAW_A_START_SET:
                            self.set_point(y, 255, 255 , 0)
            self.closedset[x] = x_count  #add x to closedset 
            del self.openset[x] # remove x from openset
            #if DEBUG_DRAW_A_START_SET:
            #   pub_marker.publish(self.markerArray)
        
        #----------------- Publish Path----------------#
        p = self.navi_goal
        while True: 
            #----- Convert idx -> Pose -------#
            step = Pose() 
            if p == current_pos_idx:
                rospy.loginfo("Finish drawing !!")
                break 
            (step.position.x ,step.position.y) = self.idx2XY(p)
            self.set_point(p, 255, 255, 255 )
            #----- Rewine ------# 
            p = self.came_from[p]
        pub_marker.publish(self.markerArray)
        self.is_need_pub = True 

    def neighbor_delta_cost(self, x ,y):
        #
        rospy.loginfo("y : " + str(y))
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

        ans =  math.sqrt(pow(dx,2) + pow(dy,2))
        return ans
    
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

    def lowest (self):
        '''
        find the lowest score in the openSet, retrun index.
        Input: Array
        output: index 
        '''
        ans = -1 
        lowest_score = float("inf")
        for i in self.openset:
            if self.openset[i] < lowest_score:
                ans = i
                lowest_score = self.openset[i]
        return ans

    def neighbor(self, x):
        '''
        return neighborhood of x, as a List with 4 node.(up, down, right, left,up-right , up-left, down-right, down-left)
        '''
        ans = list()

        # UP 
        ans.append(x + GC.width)
        # DOWN
        ans.append(x - GC.width)
        # RIGHT
        ans.append(x + 1)
        # LEFT
        ans.append(x - 1)
        # UP-RIGHT
        ans.append(x + 1 + GC.width)
        # UP-LEFT
        ans.append(x - 1 + GC.width)
        # DOWN-RIGHT
        ans.append(x + 1 - GC.width)
        # DOWN-LEFT
        ans.append(x - 1 - GC.width)
        #----------------- Boundary check ---------------# 
        ans_wihtout_exceed_boundary = list()
        for i in ans:
            if i >= 0 or i < (GC.width-1) * (GC.height-1) : # illegal range 
                ans_wihtout_exceed_boundary.append(i)
        return ans_wihtout_exceed_boundary
    
    ############################### SAME AS GLOBAL_CARTOPGRPHER ##################################
    def idx2XY (self, idx):
        '''
        idx must be interger
        '''
        origin = [GC.global_costmap.info.origin.position.x , GC.global_costmap.info.origin.position.y]

        x = (idx % GC.width) * GC.resolution + origin[0] + GC.resolution/2 # Center of point 
        # y = round(idx / width) * reso + origin[1] + reso/2 
        y = math.floor(idx / GC.width) * GC.resolution + origin[1] + GC.resolution/2 

        # print ("(x ,y ) = " + str((x,y)))
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

def main(args):

    #----- Init node ------# 
    global_planner = GLOBAL_PLANNER(foot_print)
    rospy.init_node('global_planner', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, GC.global_map_CB)
    # rospy.Subscriber('/current_position', OccupancyGrid, global_planner.current_position_CB) # Use TF to get it. 
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, global_planner.move_base_simple_goal_CB)
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
    except rospy.ROSInterruptException:
        pass

    

