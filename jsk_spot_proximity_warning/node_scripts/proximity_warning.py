#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

import PyKDL

from sound_play.msg import SoundRequest
from jsk_recognition_msgs.msg import BoundingBoxArray

from sound_play.libsoundplay import SoundClient


class ProximityWarningNode(object):

    def __init__(self):

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._frame_robot = rospy.get_param('~frame_robot', 'body')
        self._threshold_distance = rospy.get_param('~threshold_distance', 2.0)
        self._target_id_list = rospy.get_param('~target_id_list', [])
        self._duration_timeout = rospy.get_param('~duration_timeout', 0.1)

        self._sp_client = SoundClient(blocking=True,
                                      sound_action='robotsound',
                                      sound_topic='robotsound')
        self._flag_proximity = False

        rospy.Subscriber('~bbox_array', BoundingBoxArray, self.callback)

        rospy.loginfo('Node has been initialized.')

    def callback(self, msg):

        try:
            tfkdl_robot2bboxframe = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    msg.header.frame_id,
                    self._frame_robot,
                    msg.header.stamp,
                    timeout=rospy.Duration(self._duration_timeout)
                )
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('lookup failed: {}'.format(e))
            return

        flag_proximity = False

        for bbox in msg.boxes:
            if bbox.label not in self._target_id_list:
                continue
            pos_bbox_robotbased = tfkdl_robot2bboxframe * \
                PyKDL.Vector(bbox.pose.position.x,
                             bbox.pose.position.y,
                             bbox.pose.position.z)
            distance = pos_bbox_robotbased.Norm()
            if distance < self._threshold_distance:
                flag_proximity = True
                break

        self._flag_proximity = flag_proximity

    def process(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            rate.sleep()

            if self._flag_proximity:
                rospy.loginfo('Too close')
                # do something
                self._sp_client.say('Too close. Please stay away.', blocking=True)
            else:
                rospy.loginfo('There is no person near.')


def main():

    rospy.init_node('proximity_warning')

    node = ProximityWarningNode()

    node.process()


if __name__ == '__main__':
    main()
