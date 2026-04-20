#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import Pictogram
from jsk_rviz_plugins.msg import PictogramArray
from jsk_recognition_msgs.msg import BoundingBoxArray
import roslib
import tf
import turtlesim.msg
import copy
PictArr = PictogramArray()

  
def callback(data):
    PictArr.header = data.header
    PictArr.pictograms = []
    for box in data.boxes:
        msg = Pictogram()
        msg_string = Pictogram()
        msg.action = Pictogram.JUMP
        msg.speed = 0.3
        msg.header.frame_id = box.header.frame_id
        msg.pose.position = box.pose.position
        msg.pose.position.y -= 0.1
        msg.pose.position.z -= 0.1
        msg.pose.orientation.w = 0
        msg.pose.orientation.x = 0.7
        msg.pose.orientation.y = -0.7
        msg.pose.orientation.z = 0 
        msg.header.stamp = rospy.Time.now()
        msg.mode = Pictogram.PICTOGRAM_MODE
        msg.size = 0.1
        msg.color.r = 25 / 255.0
        msg.color.g = 255 / 255.0
        msg.color.b = 240 / 255.0
        msg.color.a = 1.0 
        msg.character = "chevron-thin-down"
        PictArr.pictograms.append(msg)
        msg_string = copy.deepcopy(msg)
        msg_string.pose.position.y -= 0.05
        msg_string.mode = Pictogram.PICTOGRAM_MODE
        msg_string.size = 0.1
        msg_string.character = "archive"
        PictArr.pictograms.append(msg_string)

    p.publish(PictArr)

if __name__=='__main__':
    rospy.init_node("pictogram_sample")
    p = rospy.Publisher("/pictogram", PictogramArray)
    s = rospy.Subscriber("/delivery_box_pickup/cluster_point_indices_decomposer_align_boxes_with_plane/boxes", BoundingBoxArray, callback)
    r = rospy.Rate(0.1)
    rospy.spin()
 
