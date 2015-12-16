#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import PoseStamped
import tf


def callback(boxes_msg):
    # we expect odom
    for box in boxes_msg.boxes:
        pose = box.pose
        print pose.position.z
        if 0.8 < pose.position.z and pose.position.z < 1.0:
            pose_msg = PoseStamped()
            pose_msg.header = boxes_msg.header
            pose_msg.pose = pose
            pub.publish(pose_msg)
            return
    raise Exception("something wrong")

if __name__ == "__main__":
    rospy.init_node("door_handle_box_extractor")
    # tf_listener = tf.TransformListener()
    # frame_id = rospy.get_param("~frame_id") # ground frame
    pub = rospy.Publisher("~output", PoseStamped)
    sub = rospy.Subscriber("~input", BoundingBoxArray, callback)
    rospy.spin()
