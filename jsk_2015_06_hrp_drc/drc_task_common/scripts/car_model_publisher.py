#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *    
import numpy

alpha = 0.1
rad = 0.0
car_width = 1.4
line_length = 50
def get_line_markers(rad):
    temp_velocity = numpy.array([0.1, 0.0])
    temp_point = numpy.array([0.0, 0.0])
    rotation = numpy.array([[numpy.cos(rad), -numpy.sin(rad)], [numpy.sin(rad), numpy.cos(rad)]])
    center_point_array = []
    center_velocity_array = []
    for count in range(line_length):
        temp_velocity = numpy.dot(rotation, temp_velocity)
        temp_point = temp_velocity + temp_point
        center_point_array.append(temp_point)
        center_velocity_array.append(temp_velocity)
    l_point_array = []
    r_point_array = []
    for (p, vel) in zip(center_point_array, center_velocity_array):
        vel_norm = numpy.array([vel[1], -vel[0]])
        vel_norm/=numpy.linalg.norm(vel_norm)
        l_p = p + vel_norm * (car_width/2.0)
        r_p = p - vel_norm * (car_width/2.0)
        l_point_array.append(Point(l_p[0], l_p[1], 0.4))
        r_point_array.append(Point(r_p[0], r_p[1], 0.4))
    markers = []
    marker = Marker(header=std_msgs.msg.Header(frame_id="car"), type = Marker.LINE_STRIP, action = Marker.ADD, colors = [std_msgs.msg.ColorRGBA(1, 0.3, 0, 0.5)]*line_length, scale = Vector3(0.2, 1, 1), points = l_point_array, id = 2, ns = "left_wheel")
    markers.append(marker)
    marker = Marker(header=std_msgs.msg.Header(frame_id="car"), type = Marker.LINE_STRIP, action = Marker.ADD, colors = [std_msgs.msg.ColorRGBA(1, 0.3, 0, 0.5)]*line_length, scale = Vector3(0.2, 1, 1), points = r_point_array, id = 1, ns = "right_wheel")
    markers.append(marker)
    return markers

def timer_cb(event):
    pub_marker()
def pub_marker():
    marker = Marker(ns="car_model", id=0, type=Marker.MESH_RESOURCE, action=Marker.ADD, frame_locked=True, mesh_resource = 
                    "package://gazebo_drive_simulator/models/polaris.stl"
                    ,  mesh_use_embedded_materials=False)
    marker.scale = Vector3(1, 1, 1)
    marker.header = std_msgs.msg.Header(frame_id="car", stamp=rospy.get_rostime())
    marker.pose = Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=-numpy.sqrt(2.0)/2.0, w=numpy.sqrt(2.0)/2.0))
    marker.color = std_msgs.msg.ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.7)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker_pub.publish(MarkerArray([marker]+get_line_markers(rad)))
        r.sleep()
        
def rad_cb(msg):
    global rad
    rad = msg.data
    pub_marker()
if __name__ == "__main__":
    rospy.init_node('car_model_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/car_model_marker', MarkerArray)
    rospy.Subscriber("car_degree", std_msgs.msg.Float64, rad_cb)
    rospy.Timer(rospy.Duration(0.01), timer_cb)
    rospy.spin()
