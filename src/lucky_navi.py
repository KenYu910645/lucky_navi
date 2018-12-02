#!/usr/bin/env python
from global_cartographer import GLOBAL_CARTOGRAPHER
from nav_msgs.msg import OccupancyGrid
import rospy
import time
import sys 
import math

# ----- Robot uniq parameters ------# 
# Foot print (0,0) must be the center of incribed circle 
foot_print = [[-0.57, 0.36],[0.57, 0.36],[0.57, -0.36],[-0.57, -0.36]]
        
def main(args):
    # Init something
    # v = elevator(1, 8) # 1F ~ 8F 
    global_cartographer = GLOBAL_CARTOGRAPHER(foot_print)
    rospy.init_node('global_cartographer', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, global_cartographer.global_map_CB)
    # for visualization
    pub_global_costmap = rospy.Publisher('global_costmap', OccupancyGrid ,queue_size = 10,  latch=True)
    # ev.init_viz()s

    #call at 10HZ
    r = rospy.Rate(10)
    is_publish_global_costmap = False 
    while (not rospy.is_shutdown()):
        # Draw out button 
        # pub_markers.publish(markerArray)
        if (not is_publish_global_costmap) and  global_cartographer.is_init: 
            pub_global_costmap.publish(global_cartographer.global_costmap)
            is_publish_global_costmap = True 
        else:
            pass
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass