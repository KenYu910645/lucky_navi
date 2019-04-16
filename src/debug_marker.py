#!/usr/bin/env python
import rospy
import sys
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
from utility import * 
import random 


class marker_debug():
    def __init__(self):
        #--------- ROS publisher ------------#  
        self.pub_marker_sphere = rospy.Publisher('marker_sphere', MarkerArray,queue_size = 1,latch=True)
        self.pub_marker_line   = rospy.Publisher('marker_line'  , MarkerArray,queue_size = 1,latch=True)
        self.pub_marker_arrow  = rospy.Publisher('marker_arrow' , MarkerArray,queue_size = 1,latch=True)
        
        #--------- Debug Marker Container ------------# 
        self.marker_sphere = MarkerArray()
        self.marker_line   = MarkerArray()
        self.marker_arrow  = MarkerArray()
        
        self.idx_sphere = 0 # For set_sphere 
        self.idx_line   = 0 # 
        self.idx_arrow  = 0

    def pub_sphere(self):
        self.pub_marker_sphere.publish(self.marker_sphere)
    def pub_line(self):
        self.pub_marker_line.publish(self.marker_line)
    def pub_arrow(self):
        self.pub_marker_arrow.publish(self.marker_arrow)

    def set_sphere(self, point , RGB = None  , size = 0.05):
        '''
        Set Point at MarkArray 
        Input : 
            point - (x,y) or idx 
            RGB - (r,g,b)
        '''
        point_xy = idxy_converter(point , convert_to = 'xy')
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = self.idx_sphere 
        self.idx_sphere += 1 
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        if RGB == None : 
            marker.color.r = random.randint(0,255) / 255.0
            marker.color.g = random.randint(0,255) / 255.0
            marker.color.b = random.randint(0,255) / 255.0
        else: 
            marker.color.r = RGB[0]/255.0
            marker.color.g = RGB[1]/255.0
            marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        (marker.pose.position.x , marker.pose.position.y) = point_xy
        self.marker_sphere.markers.append(marker)

    def set_line(self, points , RGB = None , size = 0.02, is_loop = True ):
        '''
        Set line at MarkArray
        Input : 
            points = [p1,p2....] 
        '''
        points_xy = idxy_converter(points , convert_to = 'xy')

        points_loop = []
        if is_loop:
            polygon = vertexs_sort_2_polygon(points)
            for idx in range(len(polygon)):
                points_loop.append(polygon[idx])
                points_loop.append(polygon[idx-1])

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = self.idx_line
        self.idx_line += 1
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0
        if RGB == None : 
            marker.color.r = random.randint(0,255) / 255.0
            marker.color.g = random.randint(0,255) / 255.0
            marker.color.b = random.randint(0,255) / 255.0
        else: 
            marker.color.r = RGB[0]/255.0
            marker.color.g = RGB[1]/255.0
            marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        for i in points_loop : 
            p = Point()
            p.x = i[0]
            p.y = i[1]
            marker.points.append(p)
        self.marker_line.markers.append(marker)

    def set_arrow (self, points, RGB = None, size = 0.05):
        '''
        Set Arrow at MarkArray
        Input:
            points = [p1 , p2]
        '''
        points_xy = idxy_converter(points , convert_to = 'xy')
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = self.idx_arrow 
        self.idx_arrow += 1 
        marker.ns = "tiles"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = size  # shaft diameter
        marker.scale.y = size*2 # head diameter
        marker.scale.z = size*2 # head length
        marker.color.a = 1.0
        if RGB == None : 
            marker.color.r = random.randint(0,255) / 255.0
            marker.color.g = random.randint(0,255) / 255.0
            marker.color.b = random.randint(0,255) / 255.0
        else: 
            marker.color.r = RGB[0]/255.0
            marker.color.g = RGB[1]/255.0
            marker.color.b = RGB[2]/255.0
        marker.pose.orientation.w = 1.0
        # ----- Set start_point and end_point -------# 
        p_start = Point()
        p_end =   Point()
        (p_start.x , p_start.y) = points_xy[0]
        (p_end.x , p_end.y)     = points_xy[1]
        marker.points.append(p_start)
        marker.points.append(p_end)
        self.marker_arrow.markers.append(marker)

    def clean_marker_sphere(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.action = marker.DELETEALL
        self.marker_sphere.markers.append(marker)
        self.pub_sphere()
        self.marker_sphere = MarkerArray()

    def clean_marker_line(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.action = marker.DELETEALL
        self.marker_line.markers.append(marker)
        self.pub_line()
        self.marker_line = MarkerArray()

    def clean_marker_arrow (self): 
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.action = marker.DELETEALL
        self.marker_arrow.markers.append(marker)
        self.pub_arrow()
        self.marker_arrow = MarkerArray()

    def clean_all_marker(self):
        self.clean_marker_sphere()
        self.clean_marker_line()
        self.clean_marker_arrow()

MARKER = marker_debug()

def main(args):
    #----- Init node ------# 
    rospy.init_node('marker_debug', anonymous=True)
    
    r = rospy.Rate(1)#call at 1HZ
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass