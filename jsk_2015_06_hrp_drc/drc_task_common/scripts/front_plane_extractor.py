#!/usr/bin/env python

# extract front plane

import rospy
from jsk_recognition_msgs.msg import PolygonArray, ModelCoefficientsArray
import message_filters
import tf

def callback(polygon_msg, coeff_msg):
    print "callback"
    # odom->ground
    max_val = -1.0;
    max_index = None
    for i in range(len(polygon_msg.polygons)):
        val = polygon_msg.likelihood
        if max_val < val:
            max_val = val
            max_index = i
    if max_index != None:
        out_poly_msg = PolygonArray()
        out_poly_msg.header = polygon_msg.header
        out_poly_msg.polygons = [polygon_msg.polygons[max_index]]
        pub_poly.publish(out_poly_msg)
        out_coeff_msg = ModelCoefficientsArray()
        out_coeff_msg.header = coeff_msg.header
        out_coeff_msg.coefficients= [coeff_msg.coefficients[max_index]]
        pub_coeff.publish(out_coeff_msg)


if __name__ == "__main__":
    rospy.init_node("front_plane_extractor")
    pub_poly = rospy.Publisher("~output_polygons", PolygonArray)
    pub_coeff = rospy.Publisher("~output_coeff", ModelCoefficientsArray)
    polygon_sub = message_filters.Subscriber('~input_polygon', PolygonArray)
    coef_sub = message_filters.Subscriber('~input_coeff', ModelCoefficientsArray)
    ts = message_filters.TimeSynchronizer([polygon_sub, coef_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()
