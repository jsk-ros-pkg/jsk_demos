#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from geometry_msgs.msg import *
import numpy

def polygon_cb(polygon_msg):
    centroid = numpy.array([0.0, 0.0, 0.0])
    point_array =[]
    for point in polygon_msg.polygon.points:
        point_array.append(numpy.array([point.x, point.y, point.z]))
        centroid += numpy.array([point.x, point.y, point.z])
    centroid /= len(polygon_msg.polygon.points)
    direction = numpy.cross((point_array[1]-point_array[0]), (point_array[2]-point_array[0]))
    direction = direction / numpy.linalg.norm(direction)
    normal_centroid_pose = Pose()
## direction to quarternion    
    if direction[2] < 0.99:
        rotation_direction = numpy.cross(numpy.array([0, 0, 1]), direction)
        rotation_direction = rotation_direction / numpy.linalg.norm(rotation_direction)    
        rotation_deg = numpy.arccos(numpy.dot(numpy.array([1, 0, 0]), direction))
        normal_centroid_pose.orientation.x = numpy.sin(rotation_deg/2)*rotation_direction[0]
        normal_centroid_pose.orientation.y = numpy.sin(rotation_deg/2)*rotation_direction[1]
        normal_centroid_pose.orientation.z = numpy.sin(rotation_deg/2)*rotation_direction[2]
        normal_centroid_pose.orientation.w = numpy.cos(rotation_deg/2)
    else:
        normal_centroid_pose.orientation.w = 1.0
## end direction to quarternion
    normal_centroid_pose.position.x = centroid[0]
    normal_centroid_pose.position.y = centroid[1]
    normal_centroid_pose.position.z = centroid[2]
    normal_centroid_pose_stamped = PoseStamped()
    normal_centroid_pose_stamped.header = polygon_msg.header
    normal_centroid_pose_stamped.pose = normal_centroid_pose
    posepub.publish(normal_centroid_pose_stamped)
if __name__ == "__main__":
    rospy.init_node('polygon_to_centroid', anonymous=True)
    posepub = rospy.Publisher('plane_centroid_pose', PoseStamped)
    rospy.Subscriber("polygon", PolygonStamped,  polygon_cb)
    rospy.spin()
