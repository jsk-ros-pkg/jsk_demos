#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import (ClusterPointIndices,
                                      BoundingBox,
                                      BoundingBoxArray)
from pcl_msgs.msg import PointIndices
import message_filters
import math

def callback(box_array, cluster_indices):
    if len(box_array.boxes) > 0:
        min_index = 0
        min_distance = 100000
        for box, i in zip(box_array.boxes, range(len(box_array.boxes))):
            distance = math.sqrt(box.pose.position.x * box.pose.position.x +
                                 box.pose.position.y * box.pose.position.y +
                                 box.pose.position.z * box.pose.position.z)
            if min_distance > distance:
                min_distance = distance
                min_index = i
        result_box_array = BoundingBoxArray()
        result_box_array.header = box_array.header
        result_box_array.boxes.append(box_array.boxes[min_index])
        pub_box_array.publish(result_box_array)
        pub_indices.publish(cluster_indices.cluster_indices[min_index])
        result_cluster_indices = ClusterPointIndices()
        result_cluster_indices.header = cluster_indices.header
        result_cluster_indices.cluster_indices = [cluster_indices.cluster_indices[min_index]]
        pub_cluster_indices.publish(result_cluster_indices)
    else:
        rospy.logwarn("No bounding box input")

if __name__ == "__main__":
   rospy.init_node("nearest_box_indices")
   pub_box_array = rospy.Publisher("~output/box_array", BoundingBoxArray)
   pub_indices = rospy.Publisher("~output/indices", PointIndices)
   pub_cluster_indices = rospy.Publisher("~output/cluster_indices",
                                         ClusterPointIndices)
   box_sub = message_filters.Subscriber('~input/box_array', BoundingBoxArray)
   indices_sub = message_filters.Subscriber("~input/indices", ClusterPointIndices)
   ts = message_filters.TimeSynchronizer([box_sub, indices_sub], 100)
   ts.registerCallback(callback)
   rospy.spin()
   
