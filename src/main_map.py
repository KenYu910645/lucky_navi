#!/usr/bin/env python
import time
from nav_msgs.msg import OccupancyGrid
import rospy 
import sys 
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
from utility import *
from map_find_corner import MAP_FIND_CORNER
from nation_unify import NATION_UNIFY
from map_spliter import MAP_SPLITER
from debug_marker import MARKER

IS_DEBUG_MSG    = True # False # True 
IS_DEBUG_MARKER = True   #  True 

#Output topic 
pub_map_find_corner = rospy.Publisher('map_find_corner', OccupancyGrid ,queue_size = 10,latch=True)
pub_map_territory   = rospy.Publisher('map_territory'  , OccupancyGrid ,queue_size = 10,latch=True)

# Classes
MFC = MAP_FIND_CORNER (is_debugMsg = IS_DEBUG_MSG, is_debugMarker = IS_DEBUG_MARKER)
MS  = MAP_SPLITER     (is_debugMsg = IS_DEBUG_MSG, is_debugMarker = IS_DEBUG_MARKER)
NU  = NATION_UNIFY    (is_debugMsg = IS_DEBUG_MSG, is_debugMarker = IS_DEBUG_MARKER)

class MAIN_MAP():
    def __init__(self):
        self.state = "stand_by" # map_find_corner # map_spliter # nation_unify # finish # timeout # error
        
    def global_map_CB(self, new_map):
        '''
        Do some judgement (TODO), decide wether to update this new map into GV  
        '''
        # Decide to update new_map into GV
        # --- Reset GV ------# 
        GV.__init__()
        #----- Refill all input ------# 
        GV.map_ori = new_map
        GV.width   = new_map.info.width
        GV.height  = new_map.info.height
        GV.reso    = new_map.info.resolution 
        #----- Set flag ---------------# 
        self.state = 'map_find_corner'
    

    def publish_map_find_corner(self):
        '''
        publish map_find_corner into /map_find_corner 
        '''
        new_map = OccupancyGrid() # Output 
        new_map.info = GV.map_ori.info
        new_map.header = GV.map_ori.header
        new_map.data = GV.map_find_corner
        pub_map_find_corner.publish(new_map)

    def publish_map_territory(self):
        '''
        publish map_territory into /map_territory
        '''
        new_map = OccupancyGrid() # Output 
        new_map.info = GV.map_ori.info
        new_map.header = GV.map_ori.header
        new_map.data = GV.map_territory
        pub_map_territory.publish(new_map)

    def iterateOnce(self):
        if self.state == 'stand_by':
            pass 
        elif self.state == 'map_find_corner':
            t_start = time.time()

            #------ Init map split -------# 
            MFC.init_map_find_corner()
            MFC.get_corner_and_contour()
            
            #------ Find aux corner -------# 
            aux_corner_list = MFC.get_aux_corner()
            for i in aux_corner_list:
                GV.map_find_corner[i] = 100


            # ----- Publish result --------# 
            self.publish_map_find_corner()
            rospy.loginfo("[map_find_corner] take " + str(time.time() - t_start) + " sec.")
            if IS_DEBUG_MSG: 
                rospy.loginfo("[map_find_corner] aux_corner : " + str(aux_corner_list))
            if IS_DEBUG_MARKER : 
                for p in aux_corner_list: 
                    MARKER.set_sphere(p, (255,0,0), size = 0.07)
                MARKER.pub_sphere()
                time.sleep(1.5)
                MARKER.clean_all_marker()
            
            self.state = 'map_spliter'
            # TMP  
            # self.state = 'finish'
        
        elif self.state == 'map_spliter':
            t_start = time.time()
            MS.init_map_territory()
            while MS.state == 'iterating':
                num = MS.iterateOnce()
                if IS_DEBUG_MARKER:
                    self.publish_map_territory()
            
            if MS.state == 'finish':
                self.publish_map_territory() 
                if IS_DEBUG_MARKER:
                    MARKER.clean_all_marker()
                    # Graph traversal
                    traversal = graph_traversal(MS.first_nation)
                    for T in traversal: 
                        paranet = T[0]
                        for child in T[1]:
                            arrow_tail = GV.nation_dict[paranet].center_of_mass
                            arrow_head = GV.nation_dict[child].center_of_mass
                            MARKER.set_arrow([arrow_tail,arrow_head], (255,0,0))
                    MARKER.pub_arrow()
                    time.sleep(3)
                    MARKER.clean_all_marker()

                rospy.loginfo("[map_spliter] Total take " + str(time.time() - t_start) + " sec.")
                self.state = "nation_unify"
                # TMP 
                # self.state = 'finish'
            elif MS.state == 'error' :
                pass  
        
        elif self.state == 'nation_unify':
            # ------ TODO -------# 
            t_start = time.time()
            NU.init_nation_unify(MS.first_nation)

            while NU.state == 'iterating':
                NU.iterateOnce()
                if IS_DEBUG_MARKER:
                    self.publish_map_territory()

            if NU.state == 'finish':
                # After unification Graph traversal
                traversal = graph_traversal(MS.first_nation)
                for T in traversal:
                    # rospy.loginfo(GV.nation_dict[T[0]].__str__())
                    paranet = T[0]
                    for child in T[1]:
                        arrow_tail = GV.nation_dict[paranet].center_of_mass
                        arrow_head = GV.nation_dict[child].center_of_mass
                        MARKER.set_arrow([arrow_tail,arrow_head],(255,0,0))
                MARKER.pub_arrow()
                self.publish_map_territory()
                MARKER.pub_sphere()
                rospy.loginfo("[nation_unify] take " + str(time.time() - t_start) + " sec.")
            elif NU.state == 'error':
                pass 
            self.state = 'finish'

        elif self.state == 'finish':
            #----- TODO -------# 
            # DO something 
            rospy.loginfo("[main_map] FINISH! ")
            self.state = 'stand_by'

#----- Declare Class -----# 
MAIN = MAIN_MAP()

def main(args):

    #----- Init node ------# 
    rospy.init_node('map_spliter', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, MAIN.global_map_CB)
    
    r = rospy.Rate(10)#call at 10HZ
    while (not rospy.is_shutdown()):
        MAIN.iterateOnce()
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass