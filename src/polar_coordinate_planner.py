#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid # Global map 
from geometry_msgs.msg import PoseArray, PoseStamped, Pose2D, Pose, Twist   # Global path 
import rospy 
import sys 
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
import tf2_ros 
from tf import transformations
from global_cartographer import GLOBAL_CARTOGRAPHER

# Parameters 
MAX_SEARCH_RADIUS = 1   # m 
MIN_SEARCH_RADIUS = 0.1 # m
SEARCH_RADIUS_RESOLUTION = 0.1 # m 
SERACH_ANGULAR_RESOLUTION = 0.017 # Rad 
FIX_PLAN_RADIUS = 1 # m
MAX_CANDIDATE_PATH_SEARCH = 50 # times

pub_marker = rospy.Publisher('markers', MarkerArray,queue_size = 1,  latch=False )
pub_goal = rospy.Publisher('/lucky_navi/goal',  Pose2D ,queue_size = 10,  latch=True)

# Tmp 
idx = 0

#----- Load paramters -----# 
foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]

GC = GLOBAL_CARTOGRAPHER(foot_print)

class POLAR_COORDINATE_PLANNER():
    def __init__(self):
        self.state = "stand_by" # planning # finish 
        self.goal = Pose2D() 
        self.current_position = Pose2D()
        # self.path = PoseArray()
        # For debug
        self.markerArray = MarkerArray()
        # 

        
    def iterateOnce(self): # State Machine 
        if   self.state == "stand_by":
            pass 
        elif self.state == "planning":
            # Debug Clean screen 
            self.clean_screen()
            # 
            dis_pos_2_goal = self.getDis2Goal(self.current_position)
            # current_position_yaw = transformations.euler_from_quaternion(self.pose_quaternion_2_list(self.current_position.pose.orientation))[2]
            
            #------ Min obstacle Test -------# 
            candidate_list = []
            min_obstacle_dis = self.get_min_obstacle_dis()
            if dis_pos_2_goal >= min_obstacle_dis:
                candidate_list.append(self.get_best_next_step(self.current_position, min_obstacle_dis))
            else: 
                candidate_list.append(self.get_best_next_step(self.current_position, dis_pos_2_goal))
            self.set_point(candidate_list[-1][0].x,candidate_list[-1][0].y,255,0,0, size = 0.05)

            '''
            # ------ GEt candidate list ---------# 
            candidate_list = [] # [[Pose2D_1 , score_1], [Pose2D_1 , score_1], .... , [Pose2D_10, score_10]]
            # for i in range (int((MAX_SEARCH_RADIUS - MIN_SEARCH_RADIUS) / SEARCH_RADIUS_RESOLUTION  + 1 )): # TODO This is a test 
            for i in range (int((dis_pos_2_goal - MIN_SEARCH_RADIUS) / SEARCH_RADIUS_RESOLUTION  + 1 )):
                search_radius = MIN_SEARCH_RADIUS + i * SEARCH_RADIUS_RESOLUTION
                candidate_list.append(self.get_best_next_step(self.current_position, search_radius))
                self.set_point(candidate_list[-1][0].x,candidate_list[-1][0].y,255,0,0, size = 0.05)
            '''
            
            '''
            # ------ Get candidate path  --------# 
            for candidate in candidate_list:
                candidate_path = [candidate] # Pose2D 
                while len(candidate_path) < MAX_CANDIDATE_PATH_SEARCH:
                    dis_pos_2_goal = self.getDis2Goal(candidate_path[-1][0])
                    next_step = [Pose2D(), -1]
                    
                    # Check arrived 
                    if dis_pos_2_goal <= FIX_PLAN_RADIUS: 
                        rospy.loginfo("ARRIVED")
                        # candidate_path.append() # TODO goal pose2D 
                        break 
                    next_step = self.get_best_next_step(candidate_path[-1][0], FIX_PLAN_RADIUS)
                    #---- Add candidate Path ------# 
                    self.set_point(next_step[0].x,next_step[0].y,0,255,0, size = 0.03)
                    candidate_path.append(next_step)
            '''
            
            pub_marker.publish(self.markerArray)


            # GOGO PULBISH
            pub_goal.publish(candidate_list[-1][0])
            # self.state = "finish"
            if dis_pos_2_goal < 0.05 : 
                self.state = "finish"

        elif self.state == "finish":
            print ("Planning Take : " + str(time.time() - self.t_start_moving) + " sec.")
            self.t_start_moving = None 
            self.state = "stand_by"
             
    
    def get_min_obstacle_dis(self):
        '''
        This is a test function 
        Go kill some ZED
        '''
        RESOLUTION = 0.05 
        r = RESOLUTION # Resulution is 0.05 too.
        while r <= 8 :  
            for i in range(720): # 0.5 degree 

                tmp = Pose2D()
                tmp.theta = i * (math.pi/360)
                (tmp.x, tmp.y) = (self.current_position.x + r*math.cos(tmp.theta) 
                                 ,self.current_position.y + r*math.sin(tmp.theta))

                tmp_cost = GC.global_costmap.data[self.XY2idx((tmp.x,tmp.y))]
                
                if tmp_cost == 100 : 
                    print ("GET MIN OBSTACLE AT : " + str(r))
                    return r 
            r += RESOLUTION 
            
        
    def get_best_next_step (self, input_pose , r):
        '''
        Calculate "pose" surrunding 360 degree , best nest step 
        Input : 
                pose - (Pose2D) where you want to start finding next step 
                r - search radius 
        Output : 
                next_step - [Pose2D_1 , score_1]
        '''
        next_step = [Pose2D(), -1] # [[Pose2D_1 , score_1], 

        for j in range (int(math.pi*2 / SERACH_ANGULAR_RESOLUTION + 1)): # 0~360
            tmp = Pose2D()
            tmp.theta = j * SERACH_ANGULAR_RESOLUTION
            (tmp.x, tmp.y) = (input_pose.x + r*math.cos(tmp.theta) 
                             ,input_pose.y + r*math.sin(tmp.theta))
            
            dis_pos_2_goal = self.getDis2Goal(input_pose)
            
            tmp_cost = 100 - GC.global_costmap.data[self.XY2idx((tmp.x,tmp.y))] # 0~100
            
            #----- Get tmp_dist  ------#
            #  tmp_dist = self.getDis2Goal(x,y) * 50 / dis_pos_2_goal # 
            if dis_pos_2_goal >= r: 
                tmp_dist = (-50.0 / r) * self.getDis2Goal(tmp) + 50*(dis_pos_2_goal + r) / r
            else: 
                tmp_dist = (-100 / (dis_pos_2_goal+r)) * self.getDis2Goal(tmp) + 100 

            #----- Get tmp_turn  ------# 
            dtheta = abs(self.angle_substitution(tmp.theta  - input_pose.theta))
            tmp_turn = 100 * (1 - dtheta / (math.pi / 2))
            if dtheta >= math.pi/2:  # TODO TODO Not enable assign backward
                tmp_turn = 0 
            
            # print ("tmp_turn: " + str(tmp_turn) )
            tmp_score = tmp_cost*0.3 + tmp_dist + tmp_turn*0.3

            self.set_point(tmp.x , tmp.y ,255*(tmp_turn/100.0),255*(tmp_turn/100.0),255*(tmp_turn/100.0))

            # print ("tmp_score: " + str(round(tmp_score))  + " = " + str(round(tmp_cost)) + " + " + str(round(tmp_dist)) + " + " + str(round(tmp_turn)))
            if tmp_score > next_step[1] :
                next_step[0] = tmp 
                #(candidate.x, candidate.y) = (x,y)
                #candidate.theta = ang 
                next_step[1] = tmp_score
        return next_step 

    
    def pose_quaternion_2_list(self, quaternion):
        """
        This function help transfer the geometry_msgs.msg.PoseStameped 
        into (translation, quaternion) <-- lists
        """
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    
    def sign (self, x):
        if x >= 0:
            return 1
        else: 
            return -1 
        
    def angle_substitution(self, ang):
        '''
        Make sure  ang is 0 ~ pi/2 or -0 ~ -pi/2  # rad 
        '''
        ans = (abs(ang) % (2* math.pi)) * self.sign(ang) # Make sure not ot exceed 360

        if ans > math.pi :
            # return (2* math.pi) - ans 
            ans =  ans - (2* math.pi) # Negative 
        elif ans < -math.pi : 
            ans =  (2* math.pi) + ans # Positive 
        else: 
            pass 
        return ans 

    def getDis2Goal(self, n):
        '''
        Get Euclidean distance between n and goal. (which both are Pose2D)
        '''
        dx = self.goal.x - n.x 
        dy = self.goal.y - n.y
        return math.sqrt(dx*dx + dy*dy)
    
    def move_base_simple_goal_CB(self, goal):
        rospy.loginfo ("Target : " + str(goal))
        # self.goal = goal # TODO Check Valid Goal, or the goal is already reached.
        (self.goal.x, self.goal.y)  = (goal.pose.position.x , goal.pose.position.y ) 
        self.goal.theta = transformations.euler_from_quaternion(self.pose_quaternion_2_list(goal.pose.orientation))[2]


        self.state = "planning"
        # TODO Do something.
        #self.reset()
        #self.navi_goal = self.XY2idx((navi_goal.pose.position.x, navi_goal.pose.position.y))
        self.t_start_moving  = time.time()
    
    def current_position_CB(self, current_position):
        # self.current_position = current_position
        (self.current_position.x, self.current_position.y)  = (current_position.pose.position.x , current_position.pose.position.y ) 
        self.current_position.theta = transformations.euler_from_quaternion(self.pose_quaternion_2_list(current_position.pose.orientation))[2]


    def set_point(self, x,y ,r ,g ,b , size = 0.02):
        '''
        Set Point at MarkArray 
        '''
        global idx 
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = idx # int(x*1000 + y*1000)
        idx += 1 
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        marker.color.r = r/255.0
        marker.color.g = g/255.0
        marker.color.b = b/255.0
        marker.pose.orientation.w = 1.0
        (marker.pose.position.x , marker.pose.position.y) = (x,y)
        self.markerArray.markers.append(marker)
    
    def clean_screen (self):
        #------- clean screen -------#
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.action = marker.DELETEALL
        self.markerArray.markers.append(marker)
        pub_marker.publish(self.markerArray)

        #------- Debug draw -------# 
        self.markerArray = MarkerArray()
    
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
#----- Declare Class -----# 
PCP = POLAR_COORDINATE_PLANNER()

def main(args):
    #----- Init node ------# 
    rospy.init_node('polar_coordinate_planner', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, PCP.move_base_simple_goal_CB) # TODO for testing 
    rospy.Subscriber('/current_position', PoseStamped, PCP.current_position_CB) 
    pub_global_costmap = rospy.Publisher('global_costmap', OccupancyGrid ,queue_size = 10,  latch=True)
    
    
    rospy.Subscriber('/map', OccupancyGrid, GC.global_map_CB)

    r = rospy.Rate(10)#call at 1HZ # TODO Test  
    while (not rospy.is_shutdown()):
        if GC.is_need_pub: 
            pub_global_costmap.publish(GC.global_costmap)
            GC.is_need_pub = False
        PCP.iterateOnce()
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
