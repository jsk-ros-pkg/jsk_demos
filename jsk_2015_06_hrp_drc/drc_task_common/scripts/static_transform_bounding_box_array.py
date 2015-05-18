#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from math import pi
from tf.transformations import *

def callback(msg):
    new_msg = BoundingBoxArray()
    new_msg.header = msg.header
    trans = concatenate_matrices(translation_matrix(transform[0:3]),
                                 euler_matrix(*transform[3:6]))
    for box in msg.boxes:
        old_pose = concatenate_matrices(translation_matrix([
            box.pose.position.x,
            box.pose.position.y,
            box.pose.position.z]),
                                        quaternion_matrix([
                                            box.pose.orientation.x,
                                            box.pose.orientation.y,
                                            box.pose.orientation.z,
                                            box.pose.orientation.w]))
        new_pose = concatenate_matrices(old_pose, trans)
        translation = translation_from_matrix(new_pose)
        rotation = quaternion_from_matrix(new_pose)
        box.pose.position.x = translation[0]
        box.pose.position.y = translation[1]
        box.pose.position.z = translation[2]
        box.pose.orientation.x = rotation[0]
        box.pose.orientation.y = rotation[1]
        box.pose.orientation.z = rotation[2]
        box.pose.orientation.w = rotation[3]
        box.dimensions.z, box.dimensions.y = box.dimensions.y, box.dimensions.z
        new_msg.boxes.append(box)
    p.publish(new_msg)
if __name__ == "__main__":
    rospy.init_node("static_transform_bounding_box_array")
    transform = rospy.get_param("~transform", [0, 0, 0, - pi / 2.0, 0, 0])
    p = rospy.Publisher("~output", BoundingBoxArray)
    s = rospy.Subscriber("~input", BoundingBoxArray, callback)
    rospy.spin()
