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

pub_marker = rospy.Publisher('markers', MarkerArray,queue_size = 1,  latch=False )

# Tmp 
idx = 0

#----- Load paramters -----# 
foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]

GC = GLOBAL_CARTOGRAPHER(foot_print)

class POLAR_COORDINATE_PLANNER():
    def __init__(self):
        self.state = "stand_by" # planning # finish 
        self.goal = None 
        self.current_position = None
        self.path = PoseArray()
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
            dis_pos_2_goal = self.getDis2Goal(self.current_position.pose.position.x ,self.current_position.pose.position.y)
            current_position_yaw = transformations.euler_from_quaternion(self.pose_quaternion_2_list(self.current_position.pose.orientation))[2]

            # ------ GEt candidate list ---------# 
            candidate_list = [] # [(x1,y1), (x2, y2), .... , (x10, y10)]
            for i in range (int((MAX_SEARCH_RADIUS - MIN_SEARCH_RADIUS) / SEARCH_RADIUS_RESOLUTION  + 1 )):
                search_radius = MIN_SEARCH_RADIUS + i * SEARCH_RADIUS_RESOLUTION
                # print ("search_radius: " + str(search_radius)) 
                candidate = None 
                candidate_score = -1 
                for i in range (int(math.pi*2 / SERACH_ANGULAR_RESOLUTION + 1)):
                    ang = i * SERACH_ANGULAR_RESOLUTION
                    # print ("ang: " + str(ang)) 
                    (x,y) = (self.current_position.pose.position.x + search_radius*math.cos(ang) , self.current_position.pose.position.y + search_radius*math.sin(ang)) 
                    
                    tmp_cost = 100 - GC.global_costmap.data[self.XY2idx((x,y))] # 0~100
                    
                    #----- Get tmp_dist  ------#
                    #  tmp_dist = self.getDis2Goal(x,y) * 50 / dis_pos_2_goal # 
                    if dis_pos_2_goal >= search_radius: 
                        tmp_dist = (-50.0 / search_radius) * self.getDis2Goal(x,y) + 50*(dis_pos_2_goal + search_radius) / search_radius
                    else: 
                        tmp_dist = (-100 / (dis_pos_2_goal+search_radius)) * self.getDis2Goal(x,y) + 100 

                    #----- Get tmp_turn  ------# 
                    dtheta = abs(self.angle_substitution(ang - current_position_yaw))
                    tmp_turn = 100 * (1 - dtheta / (math.pi / 2))
                    if dtheta >= math.pi/2:  # TODO TODO Not enable assign backward
                        tmp_turn = 0 
                    
                    # print ("tmp_turn: " + str(tmp_turn) )
                    tmp_score = tmp_cost + tmp_dist + tmp_turn*0.5
                    self.set_point(x,y,255*(tmp_turn/100.0),255*(tmp_turn/100.0),255*(tmp_turn/100.0))
                    print ("tmp_score: " + str(round(tmp_score))  + " = " + str(round(tmp_cost)) + " + " + str(round(tmp_dist)) + " + " + str(round(tmp_turn)))
                    if tmp_score > candidate_score:
                        candidate = (x,y)
                        candidate_score = tmp_score 
                self.set_point(candidate[0],candidate[1],255,0,0, size = 0.05)
                candidate_list.append(candidate)


            pub_marker.publish(self.markerArray)
            self.state = "stand_by"
        elif self.state == "finish":
            pass 
        
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
        Make sure  ang is 0 ~ pi/2 or -0 ~ -pi/2 
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

    def getDis2Goal(self, x,y):
        dx = self.goal.pose.position.x - x 
        dy = self.goal.pose.position.y - y
        return math.sqrt(dx*dx + dy*dy)
    
    def move_base_simple_goal_CB(self, goal):
        rospy.loginfo ("Target : " + str(goal))
        self.goal = goal # TODO Check Valid Goal, or the goal is already reached.
        self.state = "planning"
        # TODO Do something.
        #self.reset()
        #self.navi_goal = self.XY2idx((navi_goal.pose.position.x, navi_goal.pose.position.y))
        self.t_start_moving  = time.time()
    
    def current_position_CB(self, current_position):
        self.current_position = current_position


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

    r = rospy.Rate(10)#call at 10HZ
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
