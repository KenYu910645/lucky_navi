#!/usr/bin/env python
from global_cartographer import GLOBAL_CARTOGRAPHER
from nav_msgs.msg import OccupancyGrid
import rospy
import time
import sys 
import math

def main(args):
    # Init something
    rospy.init_node('lucky_navi', anonymous=True)
    #call at 10HZ
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass