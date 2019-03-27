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

pub_marker = rospy.Publisher('markers', MarkerArray,queue_size = 1,  latch=True  )
pub_marker_line = rospy.Publisher('marker_line', Marker ,queue_size = 1,  latch=True  )
pub_marker_arrow = rospy.Publisher('marker_arrow', MarkerArray ,queue_size = 1,  latch=True  )
pub_map_find_corner = rospy.Publisher('map_find_corner', OccupancyGrid ,queue_size = 10,  latch=True)
pub_map_territory = rospy.Publisher('map_territory', OccupancyGrid ,queue_size = 10,  latch=True)

# Classes
MFC = MAP_FIND_CORNER()
MS  = MAP_SPLITER()
NU  = NATION_UNIFY()

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
            
            self.state = 'map_spliter'
            # TMP 
            # self.state = 'finish'
        
        elif self.state == 'map_spliter':
            t_start = time.time()
            MS.init_map_territory()
            
            while MS.state == 'iterating':
                num = MS.iterateOnce()
                # --- TODO -----# 
                # per iterate debug message 
                if num != None : 
                    rospy.loginfo(GV.nation_dict[num].__str__())
                # ----Get first nation -------# 
                
                #Debug
                '''
                self.update_map_find_corner()
                self.update_map_territory() 
                pub_marker.publish(GV.markerArray)
                time.sleep(1)
                '''
            if MS.state == 'finish':
                # Graph traversal
                traversal = graph_traversal(MS.first_nation)
                for T in traversal: 
                    paranet = T[0]
                    for child in T[1]:
                        arrow_tail = GV.nation_dict[paranet].center_of_mass
                        arrow_head = GV.nation_dict[child].center_of_mass
                        set_arrow(arrow_tail , arrow_head, 255,0,0)

                pub_marker_arrow.publish(GV.marker_arrow)
                self.publish_map_territory() 
                pub_marker.publish(GV.markerArray)
                time.sleep(2)
                clean_all_marker()

                rospy.loginfo("[map_spliter] Total take " + str(time.time() - t_start) + " sec.")
                self.state = "nation_unify"
            elif MS.state == 'error' :
                pass  
        
        elif self.state == 'nation_unify':
            # ------ TODO -------# 
            t_start = time.time()
            NU.init_nation_unify(MS.first_nation)

            while NU.state == 'iterating':
                NU.iterateOnce()

            if NU.state == 'finish':
                # After unification Graph traversal
                traversal = graph_traversal(MS.first_nation)
                for T in traversal:
                    rospy.loginfo(GV.nation_dict[T[0]].__str__())
                    paranet = T[0]
                    for child in T[1]:
                        arrow_tail = GV.nation_dict[paranet].center_of_mass
                        arrow_head = GV.nation_dict[child].center_of_mass
                        set_arrow(arrow_tail , arrow_head, 255,0,0)
                
                pub_marker_arrow.publish(GV.marker_arrow)
                self.publish_map_territory()
                pub_marker.publish(GV.markerArray)
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
main_map = MAIN_MAP()

def main(args):

    #----- Init node ------# 
    rospy.init_node('map_spliter', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, main_map.global_map_CB)
    
    r = rospy.Rate(10)#call at 10HZ
    while (not rospy.is_shutdown()):
        main_map.iterateOnce()
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass