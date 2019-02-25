#!/usr/bin/env python
import time
import math
from nav_msgs.msg import OccupancyGrid
# from radius_table import radius_table
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
        self.map_split = OccupancyGrid() # Output 
        #----- debug ------# 
        self.markerArray = MarkerArray()
        #self.map = None 

    def global_map_CB(self, map):
        t_start = time.time()
        self.resolution = map.info.resolution
        self.width = map.info.width
        self.height = map.info.height
        print ("Resolution: " + str(self.resolution))
        print ("Width: " + str(self.width))
        print ("Height: " + str(self.height))
        self.map = map 
        self.map_split.info = map.info
        self.map_split.header = map.header

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
        
        pub_marker.publish(self.markerArray)
        # pub_map_split.publish(self.map_split)

        aux_corner_list = self.get_aux_coner()

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

        pub_map_split.publish(self.map_split)
        rospy.loginfo("[MS] Total take " + str(time.time() - t_start) + " sec.")


    def get_aux_coner(self):
        '''
        Find aux coner that help split map 
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
                # print ("aux_corner_list : " + str(aux_corner_list))
        return aux_corner_list




            

                    

                                



    def extend_slope (self,discover_coner_list):
        '''
        Input : 
            discover_coner_list = [corner_ori , corner_1 , corner_2]
        '''
        ans = []
        for t in range(2):
            vertex_end = discover_coner_list[0]
            vertex_start = discover_coner_list[t+1]
            (x_end, y_end) = self.idx2XY(vertex_end)
            (x_start, y_start) = self.idx2XY(vertex_start)
            #####################################
            ###   Bresenham's line algorithm  ###
            #####################################
            # ------ Find slope ---------# 
            deltax = x_end - x_start
            deltay = y_end - y_start
            if deltax == 0: # To avoid inf slope
                deltax = 0.01

            deltaerr = abs(deltay / deltax) # slope
            error = 0.0 # No error at start


            # Start extend slope 

            if deltaerr <= 1: # 0~45 degree
                y = y_end # vertex_start.y
                # for i in range(int(round(abs(deltax) / self.resolution, 0))):
                i = 1
                while True: 
                    x = x_end + i * self.resolution * self.sign(deltax)
                    idx = self.XY2idx((x,y))
                    value = self.map_split.data[idx]
                    # ---- Value judgement ------# 
                    if value == 87 or value == 100: # meet obstcle or corner : do nothing 
                        break 
                    elif value == 50: # Create new corner
                        # self.map_split.data[idx] = 99
                        ans.append(idx)
                        break
                    else: # == 0 , Update , next step 
                        error = error + deltaerr * self.resolution
                        if error >= 0.5 * self.resolution:
                            y = y + self.sign(deltay) * self.resolution
                            error = error - 1 * self.resolution
                        # Update 
                        i += 1
            else:  # 45~90 degree
                x = x_end # vertex_start.x
                deltaerr = 1 / deltaerr
                i = 1
                # for i in range(int(round(abs(deltay) / self.resolution, 0))):
                while True : 
                    y = y_end + i * self.resolution * self.sign(deltay)

                    idx = self.XY2idx((x,y))
                    value = self.map_split.data[idx]
                    # ---- Value judgement ------# 
                    if value == 87 or value == 100: # meet obstcle or corner : do nothing 
                        break 
                    elif value == 50: # Create new corner
                        # self.map_split.data[idx] = 99
                        ans.append(idx)
                        break
                    else: # == 0 , Update , next step 
                        error = error + deltaerr * self.resolution
                        if error >= 0.5 * self.resolution:
                            x = x + self.sign(deltax) * self.resolution
                            error = error - 1 * self.resolution
                        # update 
                        i += 1 
        return ans 

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