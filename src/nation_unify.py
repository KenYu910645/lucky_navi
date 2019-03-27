#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid
import rospy 
from utility import *

class NATION_UNIFY():
    def __init__(self):
        self.state = "stand_by" # finish # iterating # timeout # error 
        self.open_root = []
        self.close_root = []
    def init_nation_unify(self, first_nation ):
        self.open_root = [first_nation]
        self.close_root = [] # Already Unitied nation list
        self.state = "iterating"
    
    def iterateOnce(self):
        if self.state == "stand_by":
            pass 
        elif self.state == "finish":
            # --- TODO ----# 
            self.state = "stand_by"
        elif self.state == "iterating":
            # ------  Check finish --------# 
            if len(self.open_root) == 0:
                self.state = "finish"
                return 

            root = self.open_root.pop(0) # FIFO
            self.close_root.append(root)
            while True:
                #####################################################
                ###    Find  children, and do reachability test   ###
                #####################################################
                children_info = {}
                neighbor_nations = GV.nation_dict[root].get_neighbor_nation()

                for children in GV.nation_dict[root].get_neighbor_nation():
                    children_info[children] = {'ver_root' : None,  # [idx1, idx2] - root  vertexs_without_common_edge
                                               'ver_child': None,  # [idx1, idx2] - child vertexs_without_common_edge
                                               'area'     : None,  # float  -  Area root + children 
                                               'center'   : None,  # (x,y)  - fuse root and chilren 
                                               'varian'   : None, } # float - After fuse root and chilren
                    #----------  Get ver_root ------------# 
                    ver_root = []
                    for v in GV.nation_dict[root].vertexs:
                        if v != neighbor_nations[children][0] and v != neighbor_nations[children][1]:
                            ver_root.append(v)
                    children_info[children]['ver_root'] = ver_root
                    
                    #----------  Get ver_child ------------# 
                    ver_child = []
                    for v in GV.nation_dict[children].vertexs:
                        if v != neighbor_nations[children][0] and v != neighbor_nations[children][1]:
                            ver_child.append(v)
                    children_info[children]['ver_child'] = ver_child


                ##############################
                ####  Check reachability   ###
                ##############################
                for children in children_info.keys():
                    is_all_reachable = True 
                    for ver_child in children_info[children]['ver_child']:
                        for ver_root in children_info[children]['ver_root']:
                            if reachability_test(idx2XY(ver_child) , idx2XY(ver_root)) == False:
                                is_all_reachable = False 
                                break 
                        if not is_all_reachable: # Already Fail, try no more.
                            break 
                    if not is_all_reachable:
                        del children_info[children]
                
                #------- Check no valid --------# 
                if len(children_info) == 0:
                    break 

                ####################################
                ###   Chose from valid_children  ###
                ####################################
                (root_area , root_center) = (GV.nation_dict[root].area , GV.nation_dict[root].center_of_mass)

                for children in children_info.keys():
                    # ------- GEt area and center ----------# 
                    (children_info[children]['area'] , children_info[children]['center']) = self.fuse_center_of_mass(
                    (GV.nation_dict[children].area , GV.nation_dict[children].center_of_mass),
                    (root_area                       , root_center))

                    new_vertexs = children_info[children]['ver_root'][:]
                    new_vertexs.extend(children_info[children]['ver_child'])
                    # --------- Get distance variance ----------# 
                    children_info[children]['varian'] = self.get_distance_variance( children_info[children]['center'] ,new_vertexs)
                
                if len(GV.nation_dict[root].vertexs) > 3: #this root is unitied before
                    # Add 'do-nothing' in candidate 
                    children_info[root] = {'varian' : self.get_distance_variance(root_center , GV.nation_dict[root].vertexs) }
                
                ######################################################
                ###  Get smallest distance variance in candidate   ###
                ######################################################
                Add_this_child = None # Which nation you're going to add.

                smallest_variance  = float('inf')
                for children in children_info.keys():
                    var = children_info[children]['varian']
                    if var < smallest_variance:
                        Add_this_child = children
                        smallest_variance = var
                
                #####################
                ###  Check loner  ###
                #####################
                # check if there's a loner inside valid_children, if so , directly uniiy it.
                for children in children_info.keys():
                    if children != root : # Except  
                        child_neigh = GV.nation_dict[children].get_neighbor_nation()
                        if len(child_neigh) == 1 : # Only ONE friend, which is current ROOT.
                            if child_neigh.has_key(root):
                                Add_this_child = children
                            else: 
                                rospy.logerr("[Unitied process] Serious problem at LONER CHECK. root = " + str(root) + ", loner = " + str(children))


                #######################################
                ###  Add unitied nation into root   ###
                #######################################
                if Add_this_child == root: # End of unification
                    break
                #--------- Update vertexs -----------# 
                GV.nation_dict[root].vertexs.extend(children_info[Add_this_child]['ver_child'])
                
                #--------- Update territory
                GV.nation_dict[root].add_territory(GV.nation_dict[Add_this_child].territory)

                #--------- Update boarders -----------# 
                common_edge = neighbor_nations[Add_this_child]
                # Delete common edge
                del GV.nation_dict[root].boarders[common_edge]
                # Add new edge 
                for boarder_key in GV.nation_dict[Add_this_child].boarders.keys():
                    if boarder_key != common_edge:
                        GV.nation_dict[root].boarders[boarder_key] = GV.nation_dict[Add_this_child].boarders[boarder_key]

                #--------- Update center_of_mass and area 
                (GV.nation_dict[root].area,
                 GV.nation_dict[root].center_of_mass) = (children_info[Add_this_child]['area'],
                                                           children_info[Add_this_child]['center'])
                
                #------- Update Add_this_child's neighbor nation, ------------# 
                for child_2nd in GV.nation_dict[Add_this_child].get_neighbor_nation().keys():
                    if child_2nd == root:
                        continue
                    common_boarder_key = GV.nation_dict[child_2nd].get_neighbor_nation()[Add_this_child]
                    GV.nation_dict[child_2nd].boarders[common_boarder_key]['neigh_area'] = root 
                
                print ("------------ root:"+ str(root) + " add_in_child: "+ str(Add_this_child) +"----------------")

                #---- DEBUG -----# 



            print ("============== END OF UNIFICATION " + str(root)+" =====================")
            ########################
            ### Add open_root   ####
            ########################
            for nation in GV.nation_dict[root].get_neighbor_nation().keys():
                if nation not in self.open_root and nation not in self.close_root:
                    self.open_root.append(nation)
        

    
    def fuse_center_of_mass (self, (mass_A, center_A),(mass_B , center_B)):
        '''
        Input : 
            (mass_A , center_A)
            (mass_B , center_B)
        Output : 
            (mass_new , center_new)
        '''
        mass_new = mass_A + mass_B 
        (dx,dy) = ( - center_A[0] + center_B[0] , -center_A[1] + center_B[1])
        center_new = (center_A[0] + (dx*mass_B)/mass_new , center_A[1] + (dy*mass_B)/mass_new)

        return (mass_new , center_new)
    def get_distance_variance(self, point_ori , vertexs):
        '''
        Get distance variance from each vertex to point_ori
        Input: 
            point_ori : (x,y) - turple
            vertexs : [idx1 , idx2, .....]
        Output : 
            float - variance 
        '''
        N = len(vertexs)

        distan_ori_2_vertex = []
        for i in vertexs: 
            (vx , vy) = idx2XY(i)
            dx = vx - point_ori[0]
            dy = vy - point_ori[1]
            distan = math.sqrt(dx*dx + dy*dy)
            distan_ori_2_vertex.append(distan)
        
        #---- get avg ---------# 
        sum_dis = 0.0
        for i in distan_ori_2_vertex:
            sum_dis += i
        avg = sum_dis / float(N)

        #---- Get variance -----#
        sum_dif = 0.0
        for i in distan_ori_2_vertex:
            sum_dif += pow(i - avg , 2)
        variance = sum_dif / float(N)

        return variance