#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid
import rospy 
import sys 
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 

pub_marker = rospy.Publisher('markers', MarkerArray,queue_size = 1,  latch=True  )
pub_map_split = rospy.Publisher('map_split', OccupancyGrid ,queue_size = 10,  latch=True)
idx = 0

class MAP_SPLITER():
    def __init__(self): 
        self.is_need_pub = False # == True, if Global map is need pub.
        #---- Map info -----# 
        self.resolution = None 
        self.width = None 
        self.height = None 
        #--- output map -----# 
        self.map_split = [] # OccupancyGrid() # Output 
        self.map = OccupancyGrid() # Input  
        #----- debug ------# 
        self.markerArray = MarkerArray()
        
    def global_map_CB(self, map):
        t_start = time.time()
        self.resolution = map.info.resolution
        self.width = map.info.width
        self.height = map.info.height
        print ("Resolution: " + str(self.resolution))
        print ("Width: " + str(self.width))
        print ("Height: " + str(self.height))
        self.map = map 
        # self.map_split.info = map.info
        # self.map_split.header = map.header

        # --- Find coner ------#
        for ori_map_pixel in range(len(map.data)):
            if self.map.data[ori_map_pixel] != 0: # in obstacle 
                self.map_split.data.append(87) 
                continue
            else: # space
                neighbor_list =  self.neighbor_value(ori_map_pixel)
                # ----- Boundary check ------# 
                if len(neighbor_list) < 8 : 
                    print ("Boundary")
                    self.map_split.data.append(0)
                    continue

                coner_1 = [100 ,100 ,100, 
                           100 ,     '*',
                           100 ,'*' ,'*']
                coner_2 = [100 ,100 ,100,
                           '*' ,     100,
                           '*' ,'*' ,100]
                coner_3 = [100 ,'*' ,'*', 
                           100 ,     '*',
                           100 ,100 ,100]
                coner_4 = ['*' ,'*' ,100, 
                           '*' ,     100,
                           100 ,100 ,100]
                
                # Inner coner 
                if self.list_compare(neighbor_list, coner_1) or self.list_compare(neighbor_list, coner_2) or self.list_compare(neighbor_list, coner_3) or self.list_compare(neighbor_list, coner_4):
                    print ("Inner Coner !!")
                    self.map_split.data.append(100)
                    #(x,y) = self.idx2XY(ori_map_pixel)
                    #self.set_point(x,y,255,255,0, size = 0.1)
                    continue
                
                coner_5 = [100 ,0   ,'*', 
                           0   ,     '*',
                           '*' ,'*' ,'*']
                coner_6 = ['*' ,0   ,100,
                           '*' ,     0,
                           '*' ,'*' ,'*']
                coner_7 = ['*' ,'*' ,'*',
                           0   ,     '*',
                           100 , 0  ,'*']
                coner_8 = ['*' ,'*' ,'*', 
                           '*' ,      0,
                           '*' ,0   ,100]
                
                coner_9 = ['*' ,100   ,0, 
                           100 ,       0,
                           0   ,0     ,0]
                coner_10 = [0  ,100 ,'*', 
                            0  ,     100,
                            0  ,0   ,0  ]
                coner_11 = [0  ,0   ,  0, 
                            100,       0,
                            '*',100   ,0]
                coner_12 = [0  ,0     ,0, 
                            0  ,     100,
                            0  ,100 ,'*']
                
                if ((self.list_compare(neighbor_list, coner_5) and self.list_compare(self.neighbor_value(ori_map_pixel- 1 + self.width), coner_9)) or 
                    (self.list_compare(neighbor_list, coner_6) and self.list_compare(self.neighbor_value(ori_map_pixel+ 1 + self.width), coner_10)) or 
                    (self.list_compare(neighbor_list, coner_7) and self.list_compare(self.neighbor_value(ori_map_pixel- 1 - self.width), coner_11)) or 
                    (self.list_compare(neighbor_list, coner_8) and self.list_compare(self.neighbor_value(ori_map_pixel+ 1 - self.width), coner_12))):
                    print ("Outer Coner !!")
                    self.map_split.data.append(100)
                    #(x,y) = self.idx2XY(ori_map_pixel)
                    #self.set_point(x,y,0,255,255, size = 0.1)
                    continue
                
                is_break = False 
                for value in neighbor_list: 
                    if value == 100:# This is edge
                        self.map_split.data.append(50)
                        #(x,y) = self.idx2XY(ori_map_pixel)
                        #self.set_point(x,y,255,0,0, size = 0.05)
                        is_break = True 
                        break
                if is_break: 
                    continue

                self.map_split.data.append(0)

        #--- End finding coner and edge -----# 
        
        # pub_marker.publish(self.markerArray)

        # ---- Find aux corner
        aux_corner_list = self.get_aux_corner()

        # --- update aux corner to map_split -----# 
        new_map = OccupancyGrid() # Output 
        new_map.info = self.map_split.info
        new_map.header = self.map_split.header
        for i in range(len(self.map_split.data)):
            if i in aux_corner_list:
                new_map.data.append(100)
            else: 
                new_map.data.append(self.map_split.data[i])
        self.map_split = new_map

        open_edge = [[1437, 1443]] # edge that still need to explore # [[corner_1 ,corner_2] , [corner_3, corner_4]]
        #self.set_point(self.idx2XY(open_edge[0][0])[0],self.idx2XY(open_edge[0][0])[1],0,255,255, size = 0.05)
        #self.set_point(self.idx2XY(open_edge[0][1])[0],self.idx2XY(open_edge[0][1])[1],0,255,255, size = 0.05)
        # print (self.reachability_test(self.idx2XY(393) , self.idx2XY(417)))

        
        while len(open_edge) != 0:
            bottom_edge = open_edge.pop(0) # FIFO
            bottom_edge_slope = self.get_slope(self.idx2XY(bottom_edge[0]), self.idx2XY(bottom_edge[1]))
            normal_slope = -1.0 / bottom_edge_slope
            print ("normal_slope : " + str(normal_slope))
            # ------- Get bottom_edge_expand_list --------# 
            # dx = self.idx2XY(bottom_edge[1])[0]  - self.idx2XY(bottom_edge[0])[0]
            # print (str(self.sign(dx)))
            bottom_edge_expand_list = self.bresenham_line(self.idx2XY(bottom_edge[0]) , end_point=self.idx2XY(bottom_edge[1]))
            bottom_edge_expand_list.append(self.idx2XY(bottom_edge[0]))
            
            print (bottom_edge_expand_list)
            for i in bottom_edge_expand_list:
                self.set_point(i[0],i[1],0,255,255, size = 0.05)


            # ------ GEt posible_votex-list --------# 
            posible_votex_list = []
            for i in bottom_edge_expand_list:
                normal_line_list = self.bresenham_line(i , normal_slope, 1 , self.judgement_function_3)
                if len(normal_line_list) == 1 : 
                    normal_line_list = self.bresenham_line(i , normal_slope, -1 , self.judgement_function_3)
                
                for j in normal_line_list:
                    if self.map_split.data[self.XY2idx(j)] == 100: # Find corner
                        posible_votex_list.append(j)
            print ("")
            print ("posible_votex_list : " + str(posible_votex_list))



            votex = posible_votex_list[0]
            # print ("bottom_edge : " + str(bottom_edge))
            # print self.reachability_test(posible_votex_list[0], self.idx2XY(bottom_edge[1]))
            
            # Find a valid votex 
            votex = None
            for i in posible_votex_list:
                if self.reachability_test(i, self.idx2XY(bottom_edge[0])) and self.reachability_test(i, self.idx2XY(bottom_edge[1])):
                    votex = i
                    break
            

            print ("votex : " + str(votex))
            if votex == None:
                print ("VOTEX NOT FOUND!!")
            else: 
                self.set_point(votex[0],votex[1],255,255,255, size = 0.05)
                pub_marker.publish(self.markerArray)

            '''
            # indicate boarder 
            # boarder = [self.idx2XY(bottom_edge[0]), self.idx2XY(bottom_edge[0]), votex] # (x,y)
            number = 10 
            for i in bottom_edge_expand_list: 
                if self.map_split.data[bottom_edge_expand_list] = 

            for i in bottom_edge: 
                self.bresenham_line(self.idx2XY(i) , votex)
            '''
            
        
        pub_map_split.publish(self.map_split)
        pub_marker.publish(self.markerArray)

        rospy.loginfo("[MS] Total take " + str(time.time() - t_start) + " sec.")

    def reachability_test(self, corner_1, corner_2):
        '''
        check there're not obstacle between corner_1 and corner_2 on map_split
        Input:
            corner_1(turple - (x,y))
            corner_2(turple - (x,y))
        Output : 
            True : reachable 
            False : unreachable 
        '''
        # slope = self.get_slope(corner_1, corner_2)
        #print ("slope : " + str(slope))
        #print ("corner_1 : " + str(corner_1))
        #print ("corner_2 : " + str(corner_2))

        # dx = corner_2[0] - corner_1[0]
        #print ("dx : "+ str(dx)) 
        print ("corner_1 : " + str(corner_1))
        print ("corner_2 : " + str(corner_2))
        line = self.bresenham_line(corner_1 , end_point=corner_2)
        
        
        # Debug 
        for i in line:
            if self.map_split.data[self.XY2idx(i)] == 87: 
                self.set_point (i[0],i[1],255,0,0, size = 0.05)
            else: 
                self.set_point (i[0],i[1],0,255,255, size = 0.05)
        pub_marker.publish(self.markerArray)
        

        for i in line:
            if self.map_split.data[self.XY2idx(i)] == 87:
                return False 
        return True 

    def get_aux_corner(self):
        '''
        Find aux corner that help split map 
        extend contours slope at coners , 
        '''
        aux_corner_list = []
        for pixel_idx in range(len(self.map_split.data)):
            if self.map_split.data[pixel_idx] == 100 : # natual coner
                close_list = [] # Already go through
                open_list = [pixel_idx]  # idx that need to be explore
                discover_coner_list = [pixel_idx] # Finally become [coner_start, coner_1 , coner_2]
                while len(discover_coner_list) < 3 : # Exit when there is nothing to explore 
                    x = open_list.pop(-1) # FILO
                    close_list.append(x)
                    
                    neighbor_list = self.neighbor_value_map_split(x)

                    # Discover coner check 
                    is_discover_new_coner = False 
                    for y in neighbor_list:
                        if self.map_split.data[y] == 100 and (y not in discover_coner_list):# It's a new coner !!
                            is_discover_new_coner = True 
                            discover_coner_list.append(y)
                    
                    if not is_discover_new_coner: 
                        for y in neighbor_list:
                            if self.map_split.data[y] == 50:
                                if y not in close_list: # Not explore before, a totally new contour
                                    # print (str(y) )
                                    open_list.append(y)
                # discover_coner_list.pop(0)
                print ("============================")
                print("discover_coner_list : " + str(discover_coner_list))
                
                for i in self.extend_slope(discover_coner_list):
                    print (str(i))
                    aux_corner_list.append(i)
        return aux_corner_list
    
    def get_slope(self, start_point , end_point):
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
        print ("dx : " + str(deltax))
        print ("dy : " + str(deltay))
        if deltax == 0: # To avoid inf slope
            deltax = 0.0000001
        if deltay == 0: # To avoid 0 slope
            deltay = 0.0000001
        deltaerr = (deltay / deltax) # slope
        return deltaerr
    
    def extend_slope (self,discover_coner_list):
        '''
        Input : 
            discover_coner_list = [corner_ori , corner_1 , corner_2] (idx)
        Output : 
            ans = [aux_corner_1, aux_corner_2] (idx), len is not fix
        '''
        ans = []
        for t in range(2):
            vertex_end = discover_coner_list[0]
            vertex_start = discover_coner_list[t+1]
            (x_end, y_end) = self.idx2XY(vertex_end)
            (x_start, y_start) = self.idx2XY(vertex_start)
            
            slope  = self.get_slope(self.idx2XY(vertex_start), self.idx2XY(vertex_end))
            deltax = x_end - x_start
            line_list = self.bresenham_line((x_end, y_end), slope, self.sign(deltax) , self.judgement_function)
            
            last_idx = self.XY2idx(line_list[-1])
            if self.map_split.data[last_idx] == 50:
                ans.append(last_idx)
        return ans 
    
    def judgement_function (self, (x,y)):
        '''
        Meet non-space return True 
        '''
        value = self.map_split.data[self.XY2idx((x,y))]
        if value != 0: 
            return True 
        else: 
            return False 
    
    def judgement_function_2 (self, (x,y), end_point):
        '''
        Meet end_point return True 
        '''
        print ("self.XY2idx(end_point) : " + str(self.XY2idx(end_point)))
        print ("self.XY2idx((x,y))     : " + str(self.XY2idx((x,y))))
        if self.XY2idx(end_point) == self.XY2idx((x,y)):
            return True 
        else: 
            return False 
        
    def judgement_function_3 (self, (x,y)):
        '''
        Meet obstacle and others territory turn True 
        '''
        value = self.map_split.data[self.XY2idx((x,y))]
        if value != 0 and value != 50 and value != 100 :  # Obstacle 
            return True 
        else: 
            return False 

    
    def bresenham_line(self, start_point , slope = None  , sign = None   , end_condition = None , end_point = None ): 
        '''
        Get bresenham line by given start point with slope, will return when specify end_condition is meet.
        Input : 
            start_point : (x,y) - turple
            slope : float, recommand get from self.get_slope() 
            sign : 1 or -1 -- decide which dircetion to go , 1 means First and Fourth quadrant (x-positive position), -1 means Third and Second quadrant
            end_condition : function that return T/F
            end_point : (x,y) -- turple 
        Output : 
            List with bresenham list : [(x1,y1), (x2,y2) , (x3,y3), .....]
            Note that list not include start_point
        Note that if pass in end_point, you don't have to assig 'slope', 'sign', 'end_condition'
        '''

        ans = []
        error = 0.0
        if end_point == None : 
            dx_sign = sign
            dy_sign = self.sign(sign* slope)
            slope = abs(slope)
        else: # assign end_point
            slope = self.get_slope(start_point, end_point)
            end_condition = self.judgement_function_2
            dx_sign = self.sign(end_point[0] - start_point[0])
            dy_sign = self.sign(dx_sign * slope)
            slope = abs(slope)
            
        if 0 <= slope <= 1: # 0~45 degree
            y = start_point[1]
            i = 1
            while True:
                # Get (x,y) and update error
                x = start_point[0] + i * self.resolution * dx_sign
                error = error + slope * self.resolution
                if error >= 0.5 * self.resolution:
                    y = y + dy_sign * self.resolution
                    error = error - 1 * self.resolution

                # Append (x,y) and check end point 
                #self.set_point(x,y,0,255,255, size = 0.05)
                #pub_marker.publish(self.markerArray)
                ans.append((x,y))
                if end_point == None: 
                    if end_condition((x,y)):
                        return ans
                else: 
                    if end_condition((x,y), end_point):
                        return ans 
                # Update 
                i += 1
        else:  # 45~90 degree
            x = start_point[0]
            slope = 1 / slope # TODO will change value outside ???
            i = 1
            while True : 
                # Get (x,y) and update error
                y = start_point[1] + i * self.resolution * dy_sign
                # Update , next step 
                error = error + slope * self.resolution
                if error >= 0.5 * self.resolution:
                    x = x + dx_sign * self.resolution
                    error = error - 1 * self.resolution
                
                # Append (x,y) and check end point
                #self.set_point(x,y,0,255,255, size = 0.05)
                #pub_marker.publish(self.markerArray)
                ans.append((x,y))
                if end_point == None: 
                    if end_condition((x,y)):
                        return ans
                else: 
                    print ("end_point : " + str(end_point))
                    if end_condition((x,y), end_point):
                        return ans 
                # update 
                i += 1 

    def sign (self, x):
        if x < 0 :
            return -1
        else :
            return 1

    def list_compare(self, a , b ): 
        '''
        Note that a, b list should have same len
        support wildcard '*'

        retrun False : not same list
        return True : same list
        '''
        for i in range(len(a)):
            if a[i] == '*' or b[i] == '*' : 
                continue
            else: 
                if a[i] == b[i] : 
                    continue
                else: 
                    return False 
        return True  

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
    
    def neighbor_value_map_split(self, x):
        '''
        At map_split (100, 50 ,0)
        return neighborhood of x, as a List with 4 node.(up, down, right, left,up-right , up-left, down-right, down-left)
        '0'  '1'  '2'
        '3'  'x'  '4'
        '5'  '6'  '7'
 
        '''
        ans = list()
        # UP-LEFT
        ans.append(x - 1 + self.width)
        # UP 
        ans.append(x + self.width)
        # UP-RIGHT
        ans.append(x + 1 + self.width)
        # LEFT
        ans.append(x - 1)
        # RIGHT
        ans.append(x + 1)
        # DOWN-LEFT
        ans.append(x - 1 - self.width)
        # DOWN
        ans.append(x - self.width)
        # DOWN-RIGHT
        ans.append(x + 1 - self.width)
        return ans 
    
    def neighbor_value(self, x):
        '''
        At map (-1, 100 , 0)
        return neighborhood of x, as a List with 4 node.(up, down, right, left,up-right , up-left, down-right, down-left)
        '0'  '1'  '2'
        '3'  'x'  '4'
        '5'  '6'  '7'
 
        '''
        ans = list()

        # UP-LEFT
        ans.append(self.map.data[x - 1 + self.width])
        # UP 
        ans.append(self.map.data[x + self.width])
        # UP-RIGHT
        ans.append(self.map.data[x + 1 + self.width])
        # LEFT
        ans.append(self.map.data[x - 1])
        # RIGHT
        ans.append(self.map.data[x + 1])
        # DOWN-LEFT
        ans.append(self.map.data[x - 1 - self.width])
        # DOWN
        ans.append(self.map.data[x - self.width])
        # DOWN-RIGHT
        ans.append(self.map.data[x + 1 - self.width])


        #----------------- Boundary check ---------------# 
        ans_wihtout_exceed_boundary = list()
        for i in ans:
            if i >= 0 or i < (self.width-1) * (self.height-1) : # illegal range 
                ans_wihtout_exceed_boundary.append(i)
        return ans_wihtout_exceed_boundary

    def idx2XY (self, idx):
        '''
        idx must be interger
        '''
        origin = [self.map.info.origin.position.x , self.map.info.origin.position.y]

        x = (idx % self.width) * self.resolution + origin[0] + self.resolution/2 # Center of point 
        # y = round(idx / width) * reso + origin[1] + reso/2 
        y = math.floor(idx / self.width) * self.resolution + origin[1] + self.resolution/2 

        # print ("(x ,y ) = " + str((x,y)))
        return (x, y)

    def XY2idx(self,  XY_coor ):
        '''
        XY_coor = ( x , y)
        '''
        origin = [self.map.info.origin.position.x , self.map.info.origin.position.y]
        # Y 
        idx =  round((XY_coor[1] - origin[1]) / self.resolution - 0.5) * self.width
        # print ("Y : " +  str(idx) )
        # idx =  math.floor((XY_coor[1] - origin[1]) / reso) * width
        # X 
        idx += round((XY_coor[0] - origin[0]) / self.resolution - 0.5)
        # idx += math.floor((XY_coor[0] - origin[0]) / reso)
        # print ("idx = " + str(idx)) 
        return int(idx)

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

#----- Declare Class -----# 
MS = MAP_SPLITER()

def main(args):

    #----- Init node ------# 
    rospy.init_node('map_spliter', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, MS.global_map_CB)
    
    r = rospy.Rate(10)#call at 10HZ
    while (not rospy.is_shutdown()):
        '''
        if GC.is_need_pub: 
            pub_global_costmap.publish(GC.global_costmap)
            GC.is_need_pub = False
        if GP.is_need_pub:
            pub_global_path.publish(GP.global_path)
            GP.is_need_pub = False
        '''
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass