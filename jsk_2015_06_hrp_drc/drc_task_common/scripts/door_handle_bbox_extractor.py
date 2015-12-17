#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import PoseStamped
import tf
from tf.transformations import *

def callback(boxes_msg):
    # we expect odom
    (trans, rot) = tf_listener.lookupTransform(frame_id, boxes_msg.header.frame_id, rospy.Time(0))
    origin_matrix = concatenate_matrices(translation_matrix(trans),
                                         quaternion_matrix(rot))
    min_y_pose = None
    min_y_dist = 100.0
    min_y_box = None
    for box in boxes_msg.boxes:
        pose = box.pose
        # pose to matrix
        pose_matrix = concatenate_matrices(translation_matrix((pose.position.x, pose.position.y, pose.position.z)),
                                           quaternion_matrix(((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))))
        box_pose = concatenate_matrices(origin_matrix, pose_matrix)
        box_pos = translation_from_matrix(box_pose)
        if (box.dimensions.x < min_x or box.dimensions.x > max_x
            or box.dimensions.y < min_y or box.dimensions.y > max_y
            or box.dimensions.z < min_z or box.dimensions.z > max_z):
            continue
        print box_pos[2]
        if 0.8 < box_pos[2] and box_pos[2] < 1.0:
            print box_pos
            if min_y_dist > abs(box_pos[1]):
                min_y_dist = abs(box_pos[1])
                min_y_pose = pose
                min_y_box = box
    if min_y_pose:
        pose_msg = PoseStamped()
        pose_msg.header = boxes_msg.header
        pose_msg.pose = min_y_pose
        pub.publish(pose_msg)
        box_arr = BoundingBoxArray()
        box_arr.header = boxes_msg.header
        box_arr.boxes = [min_y_box]
        pub_box.publish(box_arr)
        return
    raise Exception("something wrong")

if __name__ == "__main__":
    rospy.init_node("door_handle_box_extractor")
    tf_listener = tf.TransformListener()
    frame_id = rospy.get_param("~frame_id") # ground frame
    pub = rospy.Publisher("~output", PoseStamped)
    pub_box = rospy.Publisher("~output_box", BoundingBoxArray)
    min_x = rospy.get_param("~min_x", 0.01)
    min_y = rospy.get_param("~min_y", 0.01)
    min_z = rospy.get_param("~min_z", 0.01)
    max_x = rospy.get_param("~max_x", 0.2)
    max_y = rospy.get_param("~max_y", 0.2)
    max_z = rospy.get_param("~max_z", 0.1)
    sub = rospy.Subscriber("~input", BoundingBoxArray, callback)
    rospy.spin()
