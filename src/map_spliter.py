#!/usr/bin/env python
import time
import math
import rospy 
import sys 
import random 

from nation_structure import NATION_STRUCTURE 
from utility import *

class MAP_SPLITER():
    def __init__(self): 
        self.is_need_pub = False # == True, if Global map is need pub.
        self.state = "stand_by" # iterating # timeout # error # finish
        self.open_edge = []
        self.first_nation = None 
    
    def init_map_territory(self):
        '''
        Given GV.map_ori , init GV.map_territory and nation_dict
        '''
        GV.map_territory = []
        GV.nation_dict = {}
        # ----- Update map_split and map_territory -------# 
        for value in GV.map_ori.data:
            if value == 100 : # obstacle
                GV.map_territory.append(100)
            elif value == -1 : # unknow
                GV.map_territory.append(100)
            else: # Value == 0 # space 
                GV.map_territory.append(0)
        # TODO ---  how to get init edge automatically.
        self.first_nation = None 
        self.open_edge = [(1437, 1467)] # [(322079, 322109)]# [(1437, 1467)] edge that still need to explore # [[corner_1 ,corner_2] , [corner_3, corner_4]]
        #                 ^ smaller   ^ bigger
        self.state = "iterating"

    def iterateOnce(self):
        '''
        iterating : return number.
        '''
        if self.state == "stand_by":
            pass 
        elif self.state == "finish":
            # TODO, do things after finish 
            self.state = "stand_by"
        
        elif self.state == "iterating":
            # ------  Check finish --------# 
            if len(self.open_edge) == 0:
                self.state = "finish"
                return 
            
            #---- get bottom edge  -------# 
            edge = self.open_edge.pop(0) # FIFO # TODO not effienent, FIFO is nessaary, data structure need change.
            
            # ------- Get number, and new a nation --------# 
            number = round(random.random()*-100) - 2
            while number in GV.nation_dict: # -127.99999999999
                number = round(random.random()*-100)

            GV.nation_dict[number] = NATION_STRUCTURE(number)
            GV.nation_dict[number].vertexs.append(edge[0]) # <- smaller
            GV.nation_dict[number].vertexs.append(edge[1]) # <- bigger

            #----- Get first nation ------# 
            if len(GV.nation_dict) == 1:
                self.first_nation = number
            
            print ("====================" + str(edge) +"=========================")
            (b1 , b2) = ( idx2XY(edge[0]) , idx2XY(edge[1])) # - turple (x,y)
            
            posible_vertex_list = self.get_posible_vertex_list(b1, b2)

            ##########################
            ###  Find best vertex  ###
            ##########################
            vertex = self.find_best_vertex([b1,b2], posible_vertex_list[0])
            if vertex == None:
                rospy.logwarn("non-claim vertex not found! start looking for claimed vertex.")
                vertex = self.find_best_vertex([b1,b2], posible_vertex_list[1], is_using_cross_over_test=True )
                if vertex == None : 
                    rospy.logerr("claim vertex not found also! Fail to find vertex.")
                    return 
            GV.nation_dict[number].vertexs.append(XY2idx(vertex))
            # TODO 
            # get boarders, by 3 vertexs.
            GV.nation_dict[number].three_vertexs_get_remain_info()
            
            #------------ Do things about open_list -------------# 
            for boarder_key in GV.nation_dict[number].boarders:
                neigh_area = GV.nation_dict[number].boarders[boarder_key]['neigh_area']
                if neigh_area == 0:
                    if boarder_key not in self.open_edge:
                        self.open_edge.append(boarder_key)
                elif neigh_area == 100 : 
                    pass 
                else: # meet other nation 
                    GV.nation_dict[neigh_area].boarders[boarder_key]['neigh_area'] = number
                    # if this edge is already in open_edge, then del it.
                    if boarder_key in self.open_edge:
                        self.open_edge.remove(boarder_key)
            return number
    
    def is_line_intersect(self , L1 ,L2):
        '''
        Input : 
            L1 : [a1,b1,c1,d1] 
            L2 : [a2,b2,c2,d2]
        Output 
            intersect_point - (x,y) - turple 
        '''
        tor = 0.000001 # e-6
        (a1,b1,c1,d1) = L1 # param - t
        (a2,b2,c2,d2) = L2 # param - s

        determinant = a1*b2 - a2*b1
        if abs(determinant) < tor:
            # print ("[is_line_intersect] determinant == 0 ,COOL")
            return [False, None]
        
        t = (a2*d1 + b2*c2 - a2*d2 - b2*c1) / determinant
        s = (b1*c2 - b1*c1 - a1*d2 + a1*d1) / determinant

        if (t < 1+tor and t > 0-tor ) and (s < 1+tor and s > 0-tor ) : # Both t,s can find intersect point at valid range 
            rc = None 
            if abs(abs(t)-1) < tor or abs(abs(t)-0) < tor or abs(abs(s)-1) < tor or abs(abs(s)-0) < tor: # intersect at end point or start point 
                rc = False 
            else: 
                rc = True 
            return [rc , (a1*t + c1 , b1*t + d1)]
        else: 
            return [False, None]
    
    def get_posible_vertex_list(self,b1,b2):
        '''
        Input: 
            b1 : bottom edge point - (x,y)
            b2 : bottom edge point - (x,y)
        Output: - list of turple - (x,y)
            [space_corner_list , occupied_corner_list ]
        '''
        ########################
        ### Get start point  ###
        ########################
        direc = self.get_unknow_direction([b1,b2])
        mid_point = ((b1[0] + b2[0]) / 2.0 , (b1[1] + b2[1]) / 2.0)
        slope_n = -1.0 / get_slope(b1, b2)
        start_search_point = bresenham_line(mid_point, slope_n, direc, ED_once_exit)

        ########################
        ### Get close list   ###
        ########################
        open_list = [XY2idx(start_search_point[0])]  # idx that need to be explore
        close_list = [] # Already go through
        
        while len(open_list) > 0 : # Exit when there is nothing to explore 
            x = open_list.pop() # FILO
            p = idx2XY(x)
            close_list.append(x)
            
            neighbor_list = neighbor_idx(x)
            neighbor_list_without_diagonal = [neighbor_list[1], neighbor_list[3], neighbor_list[4], neighbor_list[6]]# Only take 1,3,4,6 (up,down,right,left)
            for i in neighbor_list_without_diagonal:
                if GV.map_territory[i] != 100: # Make sure not obstacle 
                    if (i not in close_list) and (i not in open_list): # Make sure not calculate before 
                        if self.is_inside_search_range(b1, b2, start_search_point[0],idx2XY(i)): # Make sure is inside valid range
                            open_list.append(i)
        
        #Debug
        '''
        print ("len of close_list : " + str(len(close_list)))
        clean_screen()
        pub_marker.publish(GV.markerArray)
        for i in close_list:
            (x,y) = idx2XY(i)
            GV.markerArray = MarkerArray()
            set_point(x, y, 255,0,255, size = 0.05)
            
            pub_marker.publish(GV.markerArray)
            time.sleep(0.005)
        clean_screen()
        '''
        
        ################################
        ### Get posible_votex_list   ###
        ################################
        space_corner_list = [] 
        occupied_corner_list = []
        for i in close_list: 
            if GV.map_find_corner[i] == 100: # Is a corner
                if i != XY2idx(b1) and i != XY2idx(b2): # Not bottom edge point, ORZ # TODO this check could be skip, if you have confident that close_list won't contain b1,b2.
                    if GV.map_territory[i] == 0: # Space corner
                        set_point(idx2XY(i)[0],idx2XY(i)[1],0,255,255, size = 0.05)
                        space_corner_list.append(idx2XY(i))
                    else:  # occupied corner
                        set_point(idx2XY(i)[0],idx2XY(i)[1],255,0,0, size = 0.05)
                        occupied_corner_list.append(idx2XY(i))
                else:
                    rospy.logerr("[posible_votex_list] include bottom edge point itself. error")
        return [space_corner_list , occupied_corner_list]
    
    def find_best_vertex(self,bottom_edge_vertexs,posible_list, is_using_cross_over_test = False ):
        '''
        Check reachable test and find best vertex.
        Input : 
            bottom_edge_vertexs : [(x1,y1), (x2,y2)]
            posible_list
            is_using_cross_over_test = T/F
        Ouput : 
            vertex 
        '''
        vertex = None
        min_triangular_distance = float('inf')
        for i in posible_list:
            if reachability_test(i, bottom_edge_vertexs[0]) and reachability_test(i, bottom_edge_vertexs[1]): # valid 
                if (not is_using_cross_over_test ) or (is_using_cross_over_test and self.non_cross_over_test(bottom_edge_vertexs ,i)):
                    tmp_triangular_distance = get_len(i,bottom_edge_vertexs[0]) + get_len(i,bottom_edge_vertexs[1])
                    if min_triangular_distance > tmp_triangular_distance: # Find a current best 
                        vertex = i
                        min_triangular_distance = tmp_triangular_distance
        return vertex
    def non_cross_over_test(self, bottom_edge_vertexs ,point_t):
        '''
        Input : 
            bottom_edge_vertexs : [(x1,y1), (x2,y2)]
            point_t : (x,y)
        Ouput : 
            True : not cross over detected
            False : cross over!!!
        '''
        neighbors_idx = neighbor_5X5_idx(XY2idx(point_t))

        #-------- Which neighbor around this point.-----------# 
        near_nation_list = []
        for i in neighbors_idx:
            value = GV.map_territory[i]
            if value != 0 and value != 100: # is nation territory
                if value not in near_nation_list: 
                    near_nation_list.append(value)
        
        #-------- Get near_nation_boarders -----------# 
        near_nation_boarders = []
        for i in near_nation_list:
            near_nation_boarders.extend(GV.nation_dict[i].get_all_boarders_param_coff())
        
        
        # ------- Get test_point<->bottom_edge waist edge ---------# 
        W1 = parametric_line(bottom_edge_vertexs[0] , point_t)
        W2 = parametric_line(bottom_edge_vertexs[1] , point_t)

        # ------- Go thought test for every near_nation_boarder ------# 
        for i in near_nation_boarders:
            rc1 = self.is_line_intersect(W1, i)
            rc2 = self.is_line_intersect(W2, i)
            if (rc1[0] == True) or (rc2[0] == True):
                print ("[cross_over_test] Detect CROSS OVER")
                return False 
        return True
    def is_inside_search_range(self, b1, b2, exp ,p):
        '''
        Jugdment for searching vortex
        Input : 
            b1 : (x ,y) - bottom edge point 1
            b2 : (x ,y) - bottom edge point 2
            exp : (x,y) - example point , a point that inside search range 
            p  : (x ,y) - test point
        Ouput: 
            True  : Inside  search 
            False : Outside search
        '''
        tor = 0.000001 # e-6

        # Get dot_1
        vec_b1_b2 = [b2[0] - b1[0], b2[1] - b1[1]]# vector : b2 - b1
        vec_b1_p = [p[0]  - b1[0], p[1]  - b1[1]]# vector : p - b1
        dot_1 = vec_b1_b2[0]*vec_b1_p[0] + vec_b1_b2[1]*vec_b1_p[1]
        
        # Get dot_2
        vec_b2_b1 = [vec_b1_b2[0]*-1 , vec_b1_b2[1]*-1]
        vec_b2_p = [p[0]  - b2[0], p[1]  - b2[1]]# vector : p - b2
        dot_2 = vec_b2_b1[0]*vec_b2_p[0] + vec_b2_b1[1]*vec_b2_p[1]

        # Get vector_N
        vector_N = [vec_b1_b2[1], vec_b1_b2[0]*-1]
        vector_b1_exp = [exp[0] - b1[0] , exp[1] - b1[1]]
        vector_b2_exp = [exp[0] - b2[0] , exp[1] - b2[1]]
        dot_test_1 = vector_N[0]*vector_b1_exp[0] + vector_N[1]*vector_b1_exp[1]
        dot_test_2 = vector_N[0]*vector_b2_exp[0] + vector_N[1]*vector_b2_exp[1]
        if sign(dot_test_1) != sign(dot_test_2):
            rospy.logerr ("dot_test_1 = " + str(dot_test_1))
            rospy.logerr ("dot_test_1 = " + str(dot_test_1))
            rospy.logerr("[ERROR] Serious problem about dot_test, non-consistancy, dot_test_1 vs dot_test_2")
            return None 
        vector_N = [vector_N[0]*sign(dot_test_1) , vector_N[1]*sign(dot_test_1)]

        # Get dot_3
        dot_3 = vec_b1_p[0]*vector_N[0] + vec_b1_p[1]*vector_N[1]
        dot_4 = vec_b2_p[0]*vector_N[0] + vec_b2_p[1]*vector_N[1]
        ''' 
        if sign(dot_3) != sign(dot_4):
            rospy.logerr ("dot_3 = " + str(dot_3))
            rospy.logerr ("dot_4 = " + str(dot_4))
            rospy.logerr("[ERROR] Serious problem about dot_test, non-consistancy, dot_3 vs dot_4")
            return None 
        '''
        if dot_1 > 0-tor and dot_2 > 0-tor and dot_3 > 0+tor: # dot_1, dot_2, Accept vertical. but dot_3 can't accept vertical  
            return True 
        else: 
            return False
    def get_unknow_direction(self, edge):
        '''
        There are two side for each line, Given edge, return which side of line is the unknow_region
        Input : 
            edge: [(x_start, y_start), (x_end, y_end)]
        Output: 
            direction : 1 or -1 : throw it in bresamham line 
        '''
        bottom_edge_expand_list = bresenham_line(edge[0], end_point=edge[1])
        bottom_edge_expand_list.append(edge[0])

        normal_slope = -1.0 / get_slope(edge[0], edge[1])
        bottom_edge_side_1 = [] #  1 
        bottom_edge_side_2 = [] # -1
        direction = None 
        for p in bottom_edge_expand_list:
            bottom_edge_side_1.append(bresenham_line(p, normal_slope, 1, ED_once_exit)[0])
            bottom_edge_side_2.append(bresenham_line(p, normal_slope,-1, ED_once_exit)[0])
        rc_side_1 = tyranny_of_the_majority(bottom_edge_side_1)
        rc_side_2 = tyranny_of_the_majority(bottom_edge_side_2)

        if   rc_side_1 == 0 and rc_side_2 != 0: 
            direction = 1
        elif rc_side_1 != 0 and rc_side_2 == 0:
            direction = -1
        elif rc_side_1 != 0 and rc_side_2 != 0:
            print ("[ERROR] Both side non zero - nothing to explore?")
        else: 
            print ("[ERROR] Both side zero ")
        return direction
        # print ("direction : " + str(direction))
    

