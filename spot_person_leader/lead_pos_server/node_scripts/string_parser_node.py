#!/usr/bin/env python
# -*- coding: utf-8 -*-

import PyKDL
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

class StringParser:

    def __init__(self):

        self._frame_id_robot = rospy.get_param('~frame_id_robot', 'base_link')

        self._action_client_lead_pos = actionlib.SimpleActionClient(
                '~lead_pos',
                MoveBaseAction
                )
        self._subscriber_string = rospy.Subscriber(
                '~go_pos_string',
                String,
                self.callback
                )

    def callback(self,msg):

        rospy.loginfo('Recieved: {}'.format(msg))

        target_list = msg.data.split(' ')

        target_x = float(target_list[0]) # [m]
        target_y = float(target_list[1]) # [m]
        target_theta = float(target_list[2]) # [rad]

        target_frame = PyKDL.Frame(
                PyKDL.Rotation.RotZ(target_theta),
                PyKDL.Vector(
                    target_x,
                    target_y,
                    0
                    )
                )

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self._frame_id_robot
        goal.target_pose.pose.position.x = target_frame.p[0]
        goal.target_pose.pose.position.y = target_frame.p[1]
        goal.target_pose.pose.position.z = target_frame.p[2]
        goal.target_pose.pose.orientation.x = target_frame.M.GetQuaternion()[0]
        goal.target_pose.pose.orientation.y = target_frame.M.GetQuaternion()[1]
        goal.target_pose.pose.orientation.z = target_frame.M.GetQuaternion()[2]
        goal.target_pose.pose.orientation.w = target_frame.M.GetQuaternion()[3]

        rospy.loginfo('Sent: {}'.format(goal))
        self._action_client_lead_pos.send_goal(goal)

if __name__ == '__main__':

    rospy.init_node('string_parser')
    node = StringParser()
    rospy.spin()
