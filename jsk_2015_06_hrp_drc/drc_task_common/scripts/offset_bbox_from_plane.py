#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import (ClusterPointIndices,
                                      BoundingBox,
                                      BoundingBoxArray,
                                      PolygonArray,
                                      ModelCoefficientsArray)
from geometry_msgs.msg import (PolygonStamped, Polygon)
from pcl_msgs.msg import PointIndices
import message_filters
import math
from tf.transformations import *
import numpy

def callback(box_array, coef_array):
    # We assume that the size of box_array and polygon_array are 1
    if len(box_array.boxes) > 0 and len(coef_array.coefficients) > 0:
        if (box_array.boxes[0].header.frame_id == coef_array.coefficients[0].header.frame_id):
            box = box_array.boxes[0]
            box_pos = numpy.array([box.pose.position.x, box.pose.position.y, box.pose.position.z])
            coef = coef_array.coefficients[0].values
            n = numpy.array([coef[0], coef[1], coef[2]])
            d = (coef[0] * box_pos[0] + coef[1] * box_pos[1] + coef[2] * box_pos[2] + coef[3] /
                 math.sqrt(coef[0] * coef[0] + coef[1] * coef[1] + coef[2] * coef[2]))
            required_distance = distance - d
            rospy.loginfo("required_distance: %f" % (required_distance))
            new_box_pos = required_distance * n + box_pos
            box.pose.position.x = new_box_pos[0]
            box.pose.position.y = new_box_pos[1]
            box.pose.position.z = new_box_pos[2]
            result_box_array = BoundingBoxArray()
            result_box_array.header = box_array.header
            result_box_array.boxes = [box]
            pub_box_array.publish(result_box_array)
        else:
            rospy.logwarn("frame_id of box array and coef array are not same")
    else:
        rospy.logwarn("Size of box array and coef array are not enough")
        



if __name__ == "__main__":
   rospy.init_node("offset_bbox_from_plane")
   distance = rospy.get_param("~distance", 0.02)
   pub_box_array = rospy.Publisher("~output/box_array", BoundingBoxArray)
   box_sub = message_filters.Subscriber('~input/box_array', BoundingBoxArray)
   coef_sub = message_filters.Subscriber('~input/coefficients', ModelCoefficientsArray)
   ts = message_filters.TimeSynchronizer([box_sub, coef_sub], 100)
   ts.registerCallback(callback)
   rospy.spin()
   
