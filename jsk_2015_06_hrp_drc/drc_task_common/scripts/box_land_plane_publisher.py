#!/usr/bin/env python
import rospy
import numpy as np
PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import numpy
from jsk_recognition_msgs.msg import *
from geometry_msgs.msg import *
from pcl_msgs.msg import ModelCoefficients
from std_msgs.msg import Header
from tf import transformations
def box_array_cb(box_array):
    polygon_array = PolygonArray()
    model_coefficients_array = ModelCoefficientsArray()
    for box in box_array.boxes:
        polygon_stamped = PolygonStamped()
        coefficients = ModelCoefficients()
        quaternion = np.array([box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w])
        rotation_matrix = transformations.quaternion_matrix(quaternion)
        ux = numpy.array([rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0]])
        uy = numpy.array([rotation_matrix[0][1], rotation_matrix[1][1], rotation_matrix[2][1]])
        uz = numpy.array([rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2]])
        dim_x = box.dimensions.x/2
        dim_y = box.dimensions.y/2
        dim_z = box.dimensions.z/2
        point = box.pose.position
        for x, y in [[-dim_x, -dim_y], [dim_x, -dim_y], [dim_x, dim_y], [-dim_x, dim_y]]:
            polygon_stamped.polygon.points.append(Point32(x*ux[0]+y*uy[0]-dim_z*uz[0]+point.x,x*ux[1]+y*uy[1]-dim_z*uz[1]+point.y,x*ux[2]+y*uy[2]-dim_z*uz[2]+point.z))
        polygon_stamped.header = box.header
        polygon_array.polygons.append(polygon_stamped)
        coefficients.values = [-uz[0], -uz[1], -uz[2], ((point.x-dim_z*uz[1])*uz[0]+(point.y-dim_z*uz[1])*uz[1]+(point.z-dim_z*uz[2])*uz[2])]
        coefficients.header = box.header
        model_coefficients_array.coefficients.append(coefficients)
    polygon_array.header = box_array.header
    PArrayPub.publish(polygon_array)
    model_coefficients_array.header = box_array.header
    MArrayPub.publish(model_coefficients_array)
if __name__ == "__main__":
    rospy.init_node('calc_box_plane', anonymous=True)
    PArrayPub = rospy.Publisher('polygon_array', PolygonArray)
    MArrayPub = rospy.Publisher('model_coefficients_array', ModelCoefficientsArray)
    rospy.Subscriber("box_array", BoundingBoxArray, box_array_cb)
    rospy.spin()
