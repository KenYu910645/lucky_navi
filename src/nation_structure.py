#!/usr/bin/env python
import time
import math
import rospy 
from utility import *
from debug_marker import MARKER


class NATION_STRUCTURE(): 
    def __init__(self, number, is_debugMsg = False, is_debugMarker = False ):
        self.number = number # number # 1 to 1 numbering for every nation 
        self.vertexs = []# vertexs # [bottom_edge-1 , bottom_edge_2, vertex] - idx
        self.territory = []# territory # [idx_1, idx_2, idx_3, idx_4, .......]# Boarder not nessary to be territory, 
        self.boarders = {} # {(vertex1, vertex2) : {'XY_list' : [...original baorders..] , 'param_coff' : [a,b,c,d] , 'neigh_area' : 100}}
        self.center_of_mass = None # (x,y) - turple 
        self.area = 0
        # ---Flag -----# 
        self.is_debugMsg    = is_debugMsg
        self.is_debugMarker = is_debugMarker

    def __str__(self):
        output = '\n'
        output+= "Number : " + str(self.number) + '\n'
        output+= "vertexs: " + str(self.vertexs) + '\n'
        output+= "area: " + str(self.area) + '\n'
        output+= "center_of_mass: " + str(self.center_of_mass) + '\n'
        output+= "neighbor_nation : " + str(self.get_neighbor_nation().keys()) + '\n'
        return output 


    
    def get_triangle_area(self):
        '''
        Input : 
            triangle's three vertexs : [vertex1, vertex2 ,vertex3] - idx
        Output : 
            area of triangle 
        '''
        [v1,v2,v3] = [idx2XY(self.vertexs[0]) , idx2XY(self.vertexs[1]) , idx2XY(self.vertexs[2])]
        term_1 = v1[0]*v2[1] + v2[0]*v3[1] + v3[0]*v1[1]
        term_2 = v2[0]*v1[1] + v3[0]*v2[1] + v1[0]*v3[1]
        self.area = 0.5 * abs(term_1 - term_2) 
        return self.area

    def get_all_boarders_XY_list(self):
        '''
        Inputs : 
            Get infomation from self.boarders
        Output: 
            [[(x1,y1) , (x2,y2), ....] , [.....], [......]]
        '''
        ans = []
        for boarder_key in self.boarders:
            ans.append(self.boarders[boarder_key]['XY_list'])
        return ans
    
    def get_all_boarders_param_coff(self):
        '''
        Inputs : 
            Get infomation from self.boarders
        Output: 
            [[a,b,c,d] , [a,b,c,d], [a,b,c,d]]
        '''
        ans = []
        for boarder_key in self.boarders:
            ans.append(self.boarders[boarder_key]['param_coff'])
        return ans
    
    def get_neighbor_nation(self):
        '''
        Inputs : 
            ---
        Output: 
            {nation_number_1 : (boarders_key) , nation_number_2 : (boarders_key)}
        '''
        ans = {}
        for boarder_key in self.boarders:
            neigh_area = self.boarders[boarder_key]['neigh_area']
            if neigh_area == 0 or neigh_area == 100:
                pass
            else:
                ans[neigh_area] = boarder_key
        return ans


    def add_territory(self, new_territory_list):
        '''
        Input: 
            new_territory_list  = [idx1 , idx2 , idx3 ,.... ]
        Put these new terrtory_list at map_territory and self.territory
        '''
        for i in new_territory_list:
            # TODO Do some check, like, same idx in territory 
            self.territory.append(i)
            GV.map_territory[i] = self.number
    
    def fill_in_territory(self, start_point):
        '''
        Use dkistra way to claim a space to territory, boundary must assign at first in map_territory.
        Input : 
            start_point : idx -  where to start explore 
        Output: 
            territory: [idx1 , idx2 , idx3, .......] - not include boundary 
        '''
        close_list = [] # Already go through
        open_list = [start_point]  # idx that need to be explore
        while len(open_list) > 0 : # Exit when there is nothing to explore 
            x = open_list.pop() # FILO
            close_list.append(x)
            
            neighbor_list = neighbor_idx(x) # Only take 1,3,4,6 (up,down,right,left)
            neighbor_list_without_diagonal = [neighbor_list[1], neighbor_list[3], neighbor_list[4], neighbor_list[6]]

            for i in neighbor_list_without_diagonal: 
                if GV.map_territory[i] == 0: 
                    if (i not in close_list) and (i not in open_list): 
                        open_list.append(i)
        return close_list

    def find_crumbs(self):
        '''
        Find crubms land from a nation, Note that boarders should be claim at map_territory first.
        Input : 
            bottom_edge_expand_list : bottom_edge of the triangle, last two elements are bottom_points
        Ouptut : 
            crumbs : [idx1, idx2, .....]
        '''

        width = GV.map_ori.info.width
        crumbs = []
        abs_slope = abs(get_slope(self.vertexs[:2]))

        for i in self.boarders[(self.vertexs[0] , self.vertexs[1])]['XY_list']:
            idx = XY2idx(i)
            if abs_slope <= 1: # 0~45 degree  # Search crumbs at vertical direction
                if GV.map_territory[idx + width] == 0 and GV.map_territory[idx + width*2] == self.number:   # up
                    crumbs.append(idx + width)
                if GV.map_territory[idx - width] == 0 and GV.map_territory[idx - width*2] == self.number:   # down 
                    crumbs.append(idx - width)
            else : # 45~90 degree # Search crumbs at horizontal direction 
                if GV.map_territory[idx + 1] == 0 and GV.map_territory[idx + 2] == self.number:   # right
                    crumbs.append(idx + 1)
                if GV.map_territory[idx - 1] == 0 and GV.map_territory[idx - 2] == self.number:   # left
                    crumbs.append(idx - 1)
        return crumbs
    
    def get_diplomatic(self):
        '''
        Input: 
            boarders : [[...bottom_edge...], [...waist_edge_1...], [...waist_edge_2...]]
        Output:
            results :[[(bottom_edge_idx1, bottom_edge_idx2) , (rc_1,rc_2)],     -  bottom_edge
                      [(waist_edge_idx1,  waist_edge_idx2)  , (rc_1,rc_2)],     -  waist_edge_1
                      [(waist_edge_idx1,  waist_edge_idx2)  , (rc_1,rc_2)]]     -  waist_edge_2
            Note that edge idx are sorted, to grauntee uniquness.
        '''
        results = {}
        
        
        for boarder_key in self.boarders:
            
            slope_n = get_normal_slope(boarder_key)
            
            rc_list = []
            for direc in [1,-1]:
                side = []
                for p in self.boarders[boarder_key]['XY_list']:
                    side.extend(bresenham_line(p, slope_n, direc, ED_once_exit))
                if side == []:
                    rc_list.append(100) # boundary see as wall 
                else:
                    rc_list.append(tyranny_of_the_majority(side))
            results[boarder_key] = tuple(rc_list)


            #side_1 = []
            #side_2 = []
            #for p in self.boarders[boarder_key]['XY_list']:
            # #   side_1.extend(bresenham_line(p, slope_n, 1, ED_once_exit))
            #    side_2.extend(bresenham_line(p, slope_n,-1, ED_once_exit))
                
                # self.set_sphere(p[0],p[1],255,0,0, size = 0.05)
            #Add new open_side 
            #print ("BEfore")
            #print ("side_1 : " + str(side_1))
            #print ("side_2 : " + str(side_2))
            # (rc_1, rc_2) = (tyranny_of_the_majority(side_1),  tyranny_of_the_majority(side_2))
            #print ("After ")
            #results[boarder_key] = (rc_1, rc_2)
        return results
    
    def three_vertexs_get_remain_info(self):
        '''
        Input : 
            Using self.vertexs , to deduct remain info, such as ,boarders, territory, center_of_mass, area, neighbor_nation.
        '''
        
        for i in range(len(self.vertexs)): # 0 ,1 ,2
            # ----- GEt baorders key ------# 
            tmp = [self.vertexs[i-1] , self.vertexs[i]]
            tmp.sort()
            boarders_key = tuple(tmp)
            self.boarders[boarders_key] = {'XY_list' : None , 'param_coff' : None , 'neigh_area' : None }

            #------ Get XY_list ------# 
            self.boarders[boarders_key]['XY_list']    = expand_edge(boarders_key)
            #------ paramtric_line ----# 
            self.boarders[boarders_key]['param_coff'] = parametric_line(boarders_key)
        
        #------ Get center of mass -------# 
        self.center_of_mass = get_center_of_mass(self.vertexs)
        if self.is_debugMsg:
            print ("center_of_mass : " + str(self.center_of_mass))
        #------ Get traiangle_area -------# 
        self.area = self.get_triangle_area()
        if self.is_debugMsg:
            print ("area : " + str(self.area))
        
        #####################
        ###  territory    ###
        #####################
        #--------  Claim space boarder as territory --------#
        
        boarders_list = self.get_all_boarders_XY_list()
        territory_add =  []
        for i in boarders_list:
            for j in i :
                idx = XY2idx(j)
                if GV.map_territory[idx] == 0: # Space boarders!!
                    territory_add.append(idx)
        self.add_territory(territory_add)
        
        #--------- Fill in territory ------------# 
        territory_fill_list = self.fill_in_territory(XY2idx(self.center_of_mass))
        
        #-------- Find crumbs -----------# 
        crumbs_list = self.find_crumbs()
        territory_fill_list.extend(crumbs_list)

        #-------- Claim territory ---------# 
        self.add_territory(territory_fill_list)
        if self.is_debugMsg: 
            print ("Territory size : " + str(len(self.territory)))

        ##############################
        ###  new nation diplomatic ###
        ##############################

        results = self.get_diplomatic()
        for boarder_key in results:
            rcs = list(results[boarder_key]) # tuple -> list
            try:
                rcs.remove(self.number)
            except: 
                rospy.logerr("[diplomatic] Serious problem edges! non of the rc is nation number.results : " + str(rcs))
                return
            else: 
                self.boarders[boarder_key]['neigh_area'] = rcs[0]
                if self.is_debugMsg:
                    print ("| --> " + str(rcs[0]))
        