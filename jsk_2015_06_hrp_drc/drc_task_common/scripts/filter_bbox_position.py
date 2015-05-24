#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from drc_task_common.cfg import FilterBBoxPositionConfig
import tf
from tf.transformations import *
from jsk_recognition_msgs.msg import BoundingBoxArray

def configCallback(config, level):
    global min_z, max_z
    min_z = config.min_z
    max_z = config.max_z
    return config

def callback(msg):
    global pub, frame_id, min_z, max_z, tf_listener
    boxes = msg.boxes
    # latest
    (pos, rot) = tf_listener.lookupTransform(msg.header.frame_id, frame_id, rospy.Time(0))
    origin_pose = concatenate_matrices(
        translation_matrix(pos), quaternion_matrix(rot))
    result = BoundingBoxArray()
    result.header = msg.header
    for box in msg.boxes:
        box_pos = [box.pose.position.x, box.pose.position.y, box.pose.position.z]
        # [x, y, z, w]
        box_rot = [box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z,
                   box.pose.orientation.w]
        box_pose = concatenate_matrices(
            translation_matrix(box_pos), quaternion_matrix(box_rot))
        box_global_pose = concatenate_matrices(origin_pose, box_pose)
        box_global_pos = translation_from_matrix(box_global_pose)
        if box_global_pos[2] > min_z and box_global_pos[2] < max_z:
            result.boxes.append(box)
    pub.publish(result)

if __name__ == "__main__":
    rospy.init_node("filter_bbox_position")
    srv = Server(FilterBBoxPositionConfig, configCallback)
    frame_id = rospy.get_param("~frame_id", "ground")
    tf_listener = tf.TransformListener()
    pub = rospy.Publisher("~output", BoundingBoxArray)
    s = rospy.Subscriber("~input", BoundingBoxArray, callback)
    rospy.spin()
