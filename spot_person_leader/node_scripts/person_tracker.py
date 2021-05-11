#!/usr/bin/env python
# -*- coding: utf-8 -*-

import PyKDL
import actionlib
import rospy
import tf2_ros
import tf2_geometry_msgs

from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Bool

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)

def PersonTracker(object):

    def __init__(self):

        self._frame_id_robot = rospy.get_param('~frame_id_robot','body')
        self._label_person = rospy.get_param('~label_person',0)
        self._dist_visible = rospy.get_param('~dist_visible',1.0)
        self._timeout_transform = rospy.get_param('~timeout_transform',0.05)
        self._timeout_observation = rospy.get_param('~timeout_observation',1.0)

        self._is_person_visible = False
        self._time_observed = rospy.Time.now()

        self._sub_bbox_array = rospy.Subscriber(
                '~bbox_array',
                BoundingBoxArray,
                self._cb_bbox_array
                )
        self._pub_visible = rospy.Publisher(
                '~visible',
                Bool,
                queue_size=1
                )

    def _cb_bbox_array(self,msg):

        self._time_observed = msg.header.stamp
        frame_id_msg = msg.header.frame_id
        frame_id_robot = self._frame_id_robot

        try:
            frame_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    frame_id_robot,
                    frame_id_msg,
                    time_observed,
                    timeout=rospy.Duration(self._timeout_transform)
                )
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup transform failed. {}'.format(e))
            return

        # check if there is a person
        is_person_visible = False
        for bbox in msg.boxes:
            if bbox.label == self._label_person:
                vector_bbox = convert_msg_point_to_kdl_vector(bbox.pose.position)
                vector_bbox_robotbased = frame_fixed_to_robot * vector_bbox
                rospy.logdebug('Person found at the distance {}'.format(vector_bbox_robotbased.Norm()))
                if vector_bbox_robotbased.Norm() < self._dist_visible:
                    is_person_visible = True
                    break
        self._is_person_visible = is_person_visible

    def spin(self, spin_rate=50):

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            rate.sleep()
            if rospy.Time.now > self._time_observed + rospy.Duration(self._timeout_observation):
                self._pub_visible.publish(Bool(False))
            else:
                self._pub_visible.publish(Bool(self._is_person_visible))

def main():

    rospy.init_node('person_tracker')
    person_tracker = PersonTracker()
    person_tracker.spin()

if __name__=='__main__':
    main()
