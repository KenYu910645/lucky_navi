#!/usr/bin/env python
import time
import math
import rospy 
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
# This file Can't publish markers  


class GLOBAL_VARIABLE():
    '''
    Global variable that can access cross files.
    '''
    def __init__(self):
        # Note!!!!  Remember to update these two value, when map change (Call back)
        #--------- Input ------------#
        self.map_ori = None # Need to update
        self.width   = None
        self.height  = None
        self.reso    = None
        #--------- Output ------------# 
        self.nation_dict =     None 
        self.map_territory =   None 
        self.map_find_corner = None

GV = GLOBAL_VARIABLE() # Import GV in file, then the file can access global variables.


def sign (x):
    if x < 0 :
        return -1
    else :
        return 1

def neighbor_idx(point):
    '''
    return neighborhood of point, as a List with 8 node.(up, down, right, left,up-right , up-left, down-right, down-left)
    '0'  '1'      '2'
    '3'  'point'  '4'
    '5'  '6'      '7'
    
    Input : 
        point - idx or (x,y) 
    Ouput : 
        [idx1 , idx2 , idx3, .... ,idx8] # len should be 8 normally.
    '''
    width = GV.width
    x = idxy_converter(point , convert_to = 'idx')

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

def neighbor_5X5_idx(point):
    '''
    return neighborhood of x, as a List with 8 node.(up, down, right, left,up-right , up-left, down-right, down-left)
    '0'   '1'   '2'  '3'  '4'
    '5'   '6'   '7'  '8'  '9'
    '10'  '11'  'x'  '12' '13'
    '14'  '15'  '16' '17' '18'
    '19'  '20'  '21' '22' '23'
    
    Input : 
        point - idx or (x,y) 
    Ouput : 
        [idx1 , idx2 , idx3, .... ,idx24] # len should be 24 normally.
    '''
    width = GV.width
    x = idxy_converter(point , convert_to = 'idx')

    ans = list()
    for i in range(5):
        for j in range(5):
            width_coff = 2 - i
            const_coff = -2 + j 
            ans.append(x + width_coff*width  + const_coff)
    return ans 

def neighbor_value(point):
    '''
    At map (-1, 100 , 0)
    return neighborhood of x, as a List with 8 node.(up, down, right, left,up-right , up-left, down-right, down-left)
    '0'  '1'  '2'
    '3'  'x'  '4'
    '5'  '6'  '7'
    Input : 
        point - idx or (x,y)  
    Ouput : 
        [value1 , value2 , value3, ....] # len should be 8 normally.
    '''
    x = idxy_converter(point , convert_to = 'idx')

    neighbor_idx_list = neighbor_idx(x)
    ans = []
    for i in neighbor_idx_list:
        ans.append(GV.map_ori.data[i])
    return ans 

def get_vector_angle(vector):
    '''
    Return angle between vector and +x
    Input: 
        vector - (x,y)
    Ouptut : 
        ang  - float ()
    '''
    (x,y) = vector 
    norm = math.sqrt(x*x + y*y)
    thetasin = math.asin(y / norm)
    thetacos = math.acos(x / norm)
    thetasindegree = thetasin * 180 / math.pi
    thetacosdegree = thetacos * 180 / math.pi

    ans = None 
    if thetasindegree >= 0:
        ans = thetacosdegree
    else:
        if thetacosdegree <= 90:
            ans =  360 + thetasindegree
        else:
            ans =  180 - thetasindegree
    
    return ans * (math.pi / 180.0)

def get_slope(two_points):
    '''
    Given two points, get slope.
    Input : 
        [idx1 ,idx2] or [(x,y) , (x,y)]
    Output : 
        return slope - float
    '''
    [p1 , p2] = idxy_converter(two_points , convert_to = 'xy')

    deltax = p2[0] - p1[0]
    deltay = p2[1] - p1[1]
    if deltax == 0: # To avoid inf slope
        return float('inf')
        # deltax = 0.0000001
    if deltay == 0: # To avoid 0 slope
        return 0.0
        # deltay = 0.0000001
    return (deltay / deltax) # slope

def get_normal_slope(two_points):
    '''
    Given two points, get normal slope.
    Input : 
        [idx1 ,idx2] or [(x,y) , (x,y)]
    Output:
        return normal slope - float
    '''
    slope = get_slope(two_points)
    if slope == 0: 
        return float('inf')
    elif slope == float('inf') or slope == float('-inf'):
        return 0.0
    else:
        return -1.0 / slope

def is_in_boundary(idx) : 
    '''
    Input : 
        idx 
    Output:
        return True : inside bouandary (valid)
               False : outside bouandary (invalid)
    '''
    if idx >= 0 and idx <= GV.width*GV.height -1:
        return True 
    else: 
        return False 

def idxy_converter(data, convert_to):
    '''
    Input : 
        data : (x,y) , idx , [(x,y) , (x,y), ...] , [idx, idx,idx, ....] 
    convert_to = 'xy' or 'idx'
    if data == [] or (): 
        return data itself.
    '''
    ori_form = '' # 'xy' or 'idx'
    wrapper = None 
    try: 
        A = len(data)
    except: # idx 
        if type(data) == int:
            ori_form = 'idx'
            wrapper = None 
    else: 
        if A == 0:  # [] or ()
            rospy.logdebug("[idxy_converter] null input type, input = " + str(data))
            return data
        try: 
            B = len(data[0])
        except:
            if type(data[0]) == int : #  (idx, idx, ... ) or [idx, idx, ...]
                ori_form = 'idx'
                wrapper = type(data)
            elif type(data[0]) == float and A == 2: # (x,y)
                ori_form = 'xy'
                wrapper = None 
        else:
            if B == 2 and type(data[0]) == tuple and type(data[0][0]) == float: # [(x,y) , (x,y)] or ((x,y) , (x,y))
                ori_form = 'xy'
                wrapper = type(data)
            else: # ERROR, too many wrapper.
                pass  
    
    if ori_form == '':
        rospy.logerr("[idxy_converter] illegal input type, input = " + str(data))
        return data 
    #----- Wrapped data ------# 
    data_wrap = None 
    if wrapper == None:
        data_wrap = [data]
    else: 
        data_wrap = data

    #-------- Valid check ---------# 
    if ori_form == 'idx':
        for i in data_wrap: 
            if not is_in_boundary(i):
                rospy.logerr("[idxy_converter] data out of map range, input = " + str(i) + " > " + str(GV.width*GV.height -1))
                return data 
    elif ori_form == 'xy':
        for i in data_wrap:
            if not is_in_boundary(XY2idx(i)):
                rospy.logerr("[idxy_converter] data out of map range, input = " + str(i) + " > " + str(GV.width*GV.height -1))
                return data 

    #-------- togoogle xy and idx ---------# 
    ans = []
    if ori_form == convert_to: # same type do nothing
        ans = data_wrap[:]
    elif ori_form == 'idx' and convert_to == 'xy':
        for i in data_wrap:
            ans.append(idx2XY(i))
    elif ori_form == 'xy'  and convert_to == 'idx':
        for i in data_wrap:
            ans.append(XY2idx(i))
    output = None 
    #------ output -------# 
    if wrapper == None : 
        output =  ans[0]
    elif wrapper == tuple: 
        output = tuple(ans)
    elif wrapper == list: 
        output = ans 
    debug = "data : " + str(data) + " , " 
    debug+= ("wrapper : " + str(wrapper) + " , " )
    debug+= ("ori_form : " + str(ori_form) + " , " )
    debug+= ("output : " + str(output ) + " , " )
    # print debug 
    return output 
    
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
    # print ("idx2XY's width : " + str(width))

    origin = (GV.map_ori.info.origin.position.x , GV.map_ori.info.origin.position.y)

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

    origin = (GV.map_ori.info.origin.position.x , GV.map_ori.info.origin.position.y)
    # Y 
    idx =  round((XY_coor[1] - origin[1]) / reso - 0.5) * width
    # X 
    idx += round((XY_coor[0] - origin[0]) / reso - 0.5)
    return int(idx)

def bresenham_line(start_point , slope = None  , direc = None   , end_condition = None , point_end = None ): 
    '''
    Get bresenham line by given start point with slope, will return when specify end_condition is meet.
    Input : 
        start_point : (x,y) or idx 
        slope : float, recommand get from get_slope() 
        direc : 1 or -1 -- decide which dircetion to go , 1 means First and Fourth quadrant (x-positive position), -1 means Third and Second quadrant
        end_condition : function that return T/F
        end_point : (x,y) -- turple 
    Output : 
        List with bresenham list : [(x1,y1), (x2,y2) , (x3,y3), .....]
        Note that list not include start_point
    Note that if pass in end_point, you don't have to assig 'slope', 'sign', 'end_condition'
    '''

    point_start = idxy_converter(start_point , convert_to = 'xy')
    reso = GV.reso 
    ans = []
    error = 0.0
    if point_end == None:
        dx_sign = direc
        dy_sign = sign(direc* slope)
        slope = abs(slope)
    else: # assign end_point
        #----- Check valid assign end_point ------# 
        point_end = idxy_converter(point_end , convert_to = 'xy')
        if XY2idx(point_start) == XY2idx(point_end):
            rospy.logerr ("[bresenham_line] ERROR, point_start and end_point are the same, can't generate line." )
            return None 
        slope = get_slope([point_start, point_end])
        end_condition = ED_end_point
        dx_sign = sign(point_end[0] - point_start[0])
        dy_sign = sign(dx_sign * slope)
        slope = abs(slope)
        
    if 0 <= slope <= 1: # 0~45 degree
        y = point_start[1]
        i = 1
        while True:
            # Get (x,y) and update error
            x = point_start[0] + i * reso * dx_sign
            error = error + slope * reso
            if error >= 0.5 * reso:
                y = y + dy_sign * reso
                error = error - 1 * reso
            if is_in_boundary(XY2idx((x,y))):
                ans.append((x,y))
            else: 
                rospy.logdebug("[bresenham_line] point out of boundary.")
            if end_condition((x,y), point_end, i):
                return ans 
            # Update 
            i += 1
    else:  # 45~90 degree
        x = point_start[0]
        slope = 1 / slope
        i = 1
        while True : 
            # Get (x,y) and update error
            y = point_start[1] + i * reso * dy_sign
            # Update , next step 
            error = error + slope * reso
            if error >= 0.5 * reso:
                x = x + dx_sign * reso
                error = error - 1 * reso
            if is_in_boundary(XY2idx((x,y))):
                ans.append((x,y))
            else: 
                rospy.logdebug("[bresenham_line] point out of boundary.")
            if end_condition((x,y), point_end, i):
                return ans 
            # update 
            i += 1 

def parametric_line(points):
    '''
    x = at + c
    y = bt + d
    Input : start point and end_point 
        points = [p1, p2]
    Ouput : list
        [a,b,c,d]
    '''
    [p1_XY,  p2_XY] = idxy_converter(points, 'xy')
    (p1_idx , p2_idx) = (XY2idx(p1_XY), XY2idx(p2_XY))
    # Note that must (start_idx < end_idx) , to grantee parametric_line uniquness 
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
    ans = bresenham_line(points[0], point_end = points[1])
    ans.append(idxy_converter(points[0] , convert_to = 'xy'))
    return ans

def tyranny_of_the_majority(citizens):
    '''
    Get most often voting value on map_territory
    Input: 
        citizens : [p1, p2 , ....]
    Output:
        valule at map_territory
    '''
    voting_result = {} # {100 : 2 , 87 : 4 , ....} #value : votes 
    citizens = idxy_converter(citizens , convert_to = 'xy')

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

def get_len(two_points):
    '''
    Given two points and return line length between these points
    Input: 
        two_points = [p1,p2]
    Output: 
        length - float
    '''
    [p1 , p2] = idxy_converter(two_points , convert_to = 'xy')
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
            if (neigh_nation not in close_list) and (neigh_nation not in open_list):
                open_list.append(neigh_nation)
    return ans

def get_center_of_mass(points):
    '''
    Input : 
        pionts = [p1,p2,p3, ....]
    Output : 
        center of mass - (x,y)
    '''
    points_xy = idxy_converter(points, convert_to = 'xy')
    
    (sum_x , sum_y) = (0,0)
    for p in points_xy:
        sum_x += p[0]
        sum_y += p[1]

    center_of_mass = (sum_x / len(points), sum_y / len(points))
    return center_of_mass


def vertexs_sort_2_polygon(points):
    '''
    Input : 
        points : [p1,p2, ....]
    Output : 
        points : [p2,p1,p3, .....]
    Input and output have same length, but output is sorted.
    '''
    points_xy = idxy_converter(points , convert_to = 'xy')
    # 1. get center of mass, Pc  
    Pc = get_center_of_mass(points_xy)
    # 2. get vertor_a , points[0] - Pc
    # vector_a = (points_xy[0] - Pc[0] , points_xy[1] - Pc[1])
    vexters_dict = {}
    # vexters_dict[0] = points_xy[0] # key : value , degree : point

    for point in points_xy:
        # Get vector_b 
        vector_p = (point[0] - Pc[0] , point[1] - Pc[1])
        #dot = vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]
        #len_a = math.sqrt(vector_a[0]**2 + vector_a[1]**2)
        #len_b = math.sqrt(vector_b[0]**2 + vector_b[1]**2)
        #degree = dot / (len_a*len_b)
        degree = get_vector_angle(vector_p)
        vexters_dict[degree] = point
    
    # TODO 4. dict sort by key 
    ans = []
    items = vexters_dict.items()
    items.sort()
    for key,value in items:
        ans.append(value)
        # print key, value # print key,dict[key]
    return ans 



def reachability_test(two_points):
    '''
    check there're not obstacle between corner_1 and corner_2 on map_split
    Input:
        two_points = [p1,p2]
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
    [corner_1 , corner_2] = idxy_converter(two_points , convert_to = 'xy')
    
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