#!/usr/bin/env python
import time
import math
import rospy 
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 

class GLOBAL_VARIABLE():
    '''
    Global variable that can access cross files.
    '''
    def __init__(self):
        # Note!!!!  Remember to update these two value, when map change (Call back)
        #--------- Input ------------# 
        self.map_ori = None # Need to update  
        self.width  = None
        self.height = None
        self.reso   = None
        #--------- Output ------------# 
        self.nation_dict =     None 
        self.map_territory =   None 
        self.map_find_corner = None 
        #--------- Debug ------------# 
        self.markerArray = MarkerArray()
        self.marker_arrow = MarkerArray()
        # Debug 
        marker_line = Marker()
        marker_line.header.frame_id = "/map"
        marker_line.id = 1 
        size = 0.02
        marker_line.ns = "tiles"
        # marker.header.stamp = rospy.get_rostime()
        marker_line.type = marker_line.LINE_LIST
        marker_line.action = marker_line.ADD
        marker_line.scale.x = size
        marker_line.scale.y = size
        marker_line.scale.z = size
        marker_line.color.a = 1.0
        marker_line.color.r = 0/255.0
        marker_line.color.g = 255/255.0
        marker_line.color.b = 0/255.0
        marker_line.pose.orientation.w = 1.0
        self.marker_line = marker_line

GV = GLOBAL_VARIABLE() # Import GV in file, then the file can access global variables.



def sign (x):
    if x < 0 :
        return -1
    else :
        return 1

def neighbor_idx(x):
    '''
    return neighborhood of x, as a List with 8 node.(up, down, right, left,up-right , up-left, down-right, down-left)
    '0'  '1'  '2'
    '3'  'x'  '4'
    '5'  '6'  '7'
    
    Input : 
        x - idx 
    Ouput : 
        [idx1 , idx2 , idx3, .... ,idx8] # len should be 8 normally.
    '''
    width = GV.width

    ans = list()
    ans.append(x - 1 + width)# UP-LEFT
    ans.append(x + width)# UP 
    ans.append(x + 1 + width)# UP-RIGHT
    ans.append(x - 1)# LEFT
    ans.append(x + 1) # RIGHT
    ans.append(x - 1 - width)# DOWN-LEFT
    ans.append(x - width)# DOWN
    ans.append(x + 1 - width)# DOWN-RIGHT
    return ans 

def neighbor_5X5_idx(x):
    '''
    return neighborhood of x, as a List with 8 node.(up, down, right, left,up-right , up-left, down-right, down-left)
    '0'   '1'   '2'  '3'  '4'
    '5'   '6'   '7'  '8'  '9'
    '10'  '11'  'x'  '12' '13'
    '14'  '15'  '16' '17' '18'
    '19'  '20'  '21' '22' '23'
    
    Input : 
        x - idx 
    Ouput : 
        [idx1 , idx2 , idx3, .... ,idx24] # len should be 24 normally.
    '''
    width = GV.width

    ans = list()
    for i in range(5):
        for j in range(5):
            width_coff = 2 - i
            const_coff = -2 + j 
            ans.append(x + width_coff*width  + const_coff)
    return ans 

def neighbor_value(x):
    '''
    At map (-1, 100 , 0)
    return neighborhood of x, as a List with 8 node.(up, down, right, left,up-right , up-left, down-right, down-left)
    '0'  '1'  '2'
    '3'  'x'  '4'
    '5'  '6'  '7'
    Input : 
        x - idx 
    Ouput : 
        [value1 , value2 , value3, ....] # len should be 8 normally.
    '''
    neighbor_idx_list = neighbor_idx(x)
    ans = []
    for i in neighbor_idx_list:
        ans.append(GV.map_ori.data[i])
    return ans 

def get_slope(start_point , end_point):
    '''
    Given two points, get slope.
    Input : 
        start_point : (x,y) - tuple
        end_point = (x,y) - tuple 
    Output : 
        return slope - float
    '''
    deltax = end_point[0] - start_point[0]
    deltay = end_point[1] - start_point[1]
    if deltax == 0: # To avoid inf slope
        deltax = 0.0000001
    if deltay == 0: # To avoid 0 slope
        deltay = 0.0000001
    return (deltay / deltax) # slope

def idx2XY (idx):
    '''
    transfer map idx into (x,y) coordinate
    Input : 
        idx - must be interger , idx must inside map index.
    Output: 
        (x,y) - turple 
    '''
    width = GV.width
    reso  = GV.reso

    origin = [GV.map_ori.info.origin.position.x , GV.map_ori.info.origin.position.y]

    x = (idx % width) * reso + origin[0] + reso/2 # Center of point 
    y = math.floor(idx / width) * reso + origin[1] + reso/2 
    return (round(x,3), round(y,3)) # TODO round should be relative to resolution

def XY2idx(XY_coor):
    '''
    transfer (x,y) coordinate into  map index 
    Input : 
        XY_coor : (x,y) - turple 
    Output: 
        idx
    '''
    width = GV.width
    reso  = GV.reso

    origin = [GV.map_ori.info.origin.position.x , GV.map_ori.info.origin.position.y]
    # Y 
    idx =  round((XY_coor[1] - origin[1]) / reso - 0.5) * width
    # X 
    idx += round((XY_coor[0] - origin[0]) / reso - 0.5)
    return int(idx)

idx = 0 # For set_point 
def set_point(x,y ,r ,g ,b , size = 0.02):
    '''
    Set Point at MarkArray 
    '''
    global idx
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = idx 
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
    GV.markerArray.markers.append(marker)

def set_line(points):
    '''
    Set line at MarkArray
    Input : 
        points = [p1,p2....] 
    '''
    for i in points : 
        p = Point()
        p.x = i[0]
        p.y = i[1]
        GV.marker_line.points.append(p)

idx_arrow = 0
def set_arrow (start_point, end_point, r,g,b):
    '''
    Set Arrow at MarkArray
    Input:
        start_point:(x,y)
        end_point : (x,y) 
    '''
    global idx_arrow
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = idx_arrow 
    idx_arrow += 1 
    marker.ns = "tiles"
    marker.header.stamp = rospy.get_rostime()
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.05  # shaft diameter
    marker.scale.y = 0.1 # head diameter
    marker.scale.z = 0.1 # head length
    marker.color.a = 1.0
    marker.color.r = r/255.0
    marker.color.g = g/255.0
    marker.color.b = b/255.0
    marker.pose.orientation.w = 1.0
    p_start = Point()
    p_end =   Point()
    (p_start.x , p_start.y) = start_point
    (p_end.x , p_end.y)     = end_point
    marker.points.append(p_start)
    marker.points.append(p_end)
    # (marker.pose.position.x , marker.pose.position.y) = (x,y)
    GV.marker_arrow.markers.append(marker)


def clean_marker_ball():
    #------- clean screen -------#
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.action = marker.DELETEALL
    GV.markerArray.markers.append(marker)
    # pub_marker.publish(self.markerArray)

def clean_marker_line():
    # clean_screen()
    GV.marker_line = Marker()
    # GV.marker_arrow = Marker()

def clean_marker_arrow (): 
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.action = marker.DELETEALL
    GV.marker_arrow.markers.append(marker)

def clean_all_marker():
    clean_marker_ball()
    clean_marker_line()
    clean_marker_arrow()


def bresenham_line(start_point , slope = None  , direc = None   , end_condition = None , end_point = None ): 
    '''
    Get bresenham line by given start point with slope, will return when specify end_condition is meet.
    Input : 
        start_point : (x,y) - turple
        slope : float, recommand get from get_slope() 
        direc : 1 or -1 -- decide which dircetion to go , 1 means First and Fourth quadrant (x-positive position), -1 means Third and Second quadrant
        end_condition : function that return T/F
        end_point : (x,y) -- turple 
    Output : 
        List with bresenham list : [(x1,y1), (x2,y2) , (x3,y3), .....]
        Note that list not include start_point
    Note that if pass in end_point, you don't have to assig 'slope', 'sign', 'end_condition'
    '''
    reso = GV.reso 

    ans = []
    error = 0.0
    if end_point == None :
        dx_sign = direc
        dy_sign = sign(direc* slope)
        slope = abs(slope)
    else: # assign end_point
        #----- Check valid assign end_point ------# 
        if XY2idx(start_point) == XY2idx(end_point):
            print ("[bresenham_line] ERROR, start_point and end_point are the same, can't generate line." )
            return None 
        slope = get_slope(start_point, end_point)
        end_condition = ED_end_point
        dx_sign = sign(end_point[0] - start_point[0])
        dy_sign = sign(dx_sign * slope)
        slope = abs(slope)
        
    if 0 <= slope <= 1: # 0~45 degree
        y = start_point[1]
        i = 1
        while True:
            # Get (x,y) and update error
            x = start_point[0] + i * reso * dx_sign
            error = error + slope * reso
            if error >= 0.5 * reso:
                y = y + dy_sign * reso
                error = error - 1 * reso

            # Append (x,y) and check end point 
            #self.set_point(x,y,0,255,255, size = 0.05)
            #pub_marker.publish(self.markerArray)
            ans.append((x,y))
            if end_condition((x,y), end_point, i):
                return ans 
            # Update 
            i += 1
    else:  # 45~90 degree
        x = start_point[0]
        slope = 1 / slope # TODO will change value outside ???
        i = 1
        while True : 
            # Get (x,y) and update error
            y = start_point[1] + i * reso * dy_sign
            # Update , next step 
            error = error + slope * reso
            if error >= 0.5 * reso:
                x = x + dx_sign * reso
                error = error - 1 * reso
            
            # Append (x,y) and check end point
            #self.set_point(x,y,0,255,255, size = 0.05)
            #pub_marker.publish(self.markerArray)
            ans.append((x,y))
            if end_condition((x,y), end_point, i):
                return ans 
            # update 
            i += 1 

def parametric_line(p1_XY, p2_XY):
    '''
    x = at + c
    y = bt + d
    Input : start point and end_point 
        points = (start_idx, end_idx)
        Note that must (start_idx < end_idx) , to grantee parametric_line uniquness 
    Ouput : list
        [a,b,c,d]
    '''
    
    (p1_idx , p2_idx) = (XY2idx(p1_XY), XY2idx(p2_XY))
    if p1_idx < p2_idx:
        p_s = p1_XY
        p_e = p2_XY
    elif p1_idx > p2_idx:
        p_s = p2_XY
        p_e = p1_XY
    else: # idx1 == idx2
        rospy.logerr("[parametric_line] ERROR, start_point == end_point!")
        return None 
    #       [      a       ]  [       b     ]   [  c  ]  [  d  ]
    return [p_e[0] - p_s[0] , p_e[1] - p_s[1] , p_s[0] , p_s[1]]



def expand_edge(points):
    '''
    Given two points, return edge.
    Input:
        (point_s_idx , point_e_idx) - turple
    Output: 
        [(x1,y1), (x2,y2), ....... point_e, point_s] - last two elements are start point and end_point
    '''
    point_s = idx2XY(points[0])
    point_e = idx2XY(points[1])
    ans = bresenham_line(point_s, end_point = point_e)
    ans.append(point_s)
    return ans

def tyranny_of_the_majority(citizens):
    '''
    Get most often voting value on map_territory
    Input: 
        citizens : [(x1,y1), (x2,y2), ......]
    Output:
        valule at map_territory
    '''
    voting_result = {} # {100 : 2 , 87 : 4 , ....} #value : votes 
    for i in citizens:
        value = GV.map_territory[XY2idx(i)]
        if value in voting_result: 
            voting_result[value] += 1
        else: 
            voting_result[value]  = 1
    
    # Counting voting result
    most_voting_value = None 
    most_vote = -1 
    for i in voting_result:
        if voting_result[i] > most_vote: 
            most_voting_value = i
            most_vote = voting_result[i]
    return most_voting_value

def get_len(p1 , p2):
    '''
    Given two points and return line length between these points
    Input: 
        p1 - (x,y) - turple
        p2 - (x,y) - turple
    Output: 
        length - float
    '''
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx*dx + dy*dy)

def graph_traversal(start_nation):
    '''
    Input : 
        start_nation : nation_number 
    Output : 
        ans  : [ (start_nation , (n1,n2) ) , (n2 , (n1,n3) ), (n2 , (n1) )......]
    '''
    ans = []
    close_list = [] # Already go through
    open_list = [start_nation]  # idx that need to be explore
    while len(open_list) > 0 : # Exit when there is nothing to explore 
        x = open_list.pop() # FILO
        close_list.append(x)

        ans.append((x , tuple(GV.nation_dict[x].get_neighbor_nation().keys())))

        for neigh_nation  in GV.nation_dict[x].get_neighbor_nation().keys():
            '''
            # Do something 
            print ( str(x) + "->" + str(neigh_nation))
            arrow_tail = self.nation_dict[x].center_of_mass
            arrow_head = self.nation_dict[neigh_nation].center_of_mass
            set_arrow(arrow_tail , arrow_head, 255,0,0)
            '''
            if (neigh_nation not in close_list) and (neigh_nation not in open_list):
                open_list.append(neigh_nation)
    return ans


def reachability_test(corner_1, corner_2):
    '''
    check there're not obstacle between corner_1 and corner_2 on map_split
    Input:
        corner_1(turple - (x,y))
        corner_2(turple - (x,y))
    Output : 
        True : reachable 
        False : unreachable 
    '''
    '''
    # Use bresenham line to check if there's obstacle between two corner
    line = bresenham_line(corner_1 , end_point=corner_2)
    for i in line:
        if MFC.map_find_corner[XY2idx(i)] == 87:
            return False 
    return True
    '''
    if XY2idx(corner_1) == XY2idx(corner_2):
        rospy.logerr("[reachability_test] corner_1 == corner_1")
        return True 

    tor = 0.000001 # e-6
    close_list = [] # Already go through
    open_list = [XY2idx(corner_1)]  # idx that need to be explore
    
    vector_1to2 = [corner_2[0] - corner_1[0] , corner_2[1] - corner_1[1]]
    vector_N = [vector_1to2[1], vector_1to2[0]*-1]
    vector_N_len = math.sqrt(vector_N[0] * vector_N[0] + vector_N[1] * vector_N[1])
    unit_vector_N = [vector_N[0] / vector_N_len , vector_N[1] / vector_N_len]

    while len(open_list) > 0 : # Exit when there is nothing to explore 
        x = open_list.pop() # FILO
        close_list.append(x)
        
        # Debug 
        #(px,py) = idx2XY(x)
        #set_point(px, py , 255,0,255, size=0.05)

        # Check reached goal 
        if x == XY2idx(corner_2):
            return True 

        neighbor_list = neighbor_idx(x)

        for i in neighbor_list:
            if GV.map_find_corner[i] != 87: # Is not wall
                # Calculate vector_p
                (px,py) = idx2XY(i)
                vector_1_to_p = [px - corner_1[0] , py - corner_1[1]]
                # inner product
                dot = abs(unit_vector_N[0] * vector_1_to_p[0] + unit_vector_N[1] * vector_1_to_p[1])
                # Check in range 
                if dot <= 0.05 + tor:
                    if (i not in close_list) and (i not in open_list): 
                        open_list.append(i)
    return False



###################################
###   Bresemham end condition   ###
###################################
def ED_once_exit ((x,y),  end_point, times):
    '''
    end_condition for bresenham_line : only execute loop once , then exit.
    So Output for bresemham centainly have length 1.
    '''
    if times >= 1 : 
        return True 
    else: 
        return False 

def ED_end_point ((x,y), end_point, times):
    '''
    Meet end_point return True 
    '''
    if XY2idx(end_point) == XY2idx((x,y)):
        return True 
    else: 
        return False 