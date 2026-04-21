#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import PyKDL
import actionlib
import rospy
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from geometry_msgs.msg import PoseStamped

from spot_ros_client.libspotros import SpotRosClient
from sound_play.libsoundplay import SoundClient


class LeadPosServer:

    def __init__(self):

        self._frame_id_robot = rospy.get_param('~frame_id_robot', 'body')
        self._frame_id_fixed = rospy.get_param('~frame_id_fixed', 'odom')
        self._timeout_transform = rospy.get_param('~timeout_transform', 0.05)

        self._tolerance_x = rospy.get_param('~tolerance_x', 0.1)
        self._tolerance_y = rospy.get_param('~tolerance_y', 0.1)
        self._tolerance_theta = rospy.get_param('~tolerance_theta', 0.1)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._spot_client = SpotRosClient()
        self._sound_client = SoundClient()

        self._state_person_visible = False
        self._state_last_visible = rospy.Time()
        self._state_duration_visible = rospy.Duration()

        self._subscriber_person_visible = rospy.Subscriber(
            '~visible',
            Bool,
            self.callback_person_view
        )
        self._action_server_lead_pos = actionlib.SimpleActionServer(
            '~lead_pos',
            MoveBaseAction,
            execute_cb=self.handler_lead_pose,
            auto_start=True
        )

        rospy.loginfo('Initialized')

    def callback_person_view(self, msg):

        if self._state_person_visible != msg.data:
            self._state_last_visible = rospy.Time.now()
            self._state_duration_visible = rospy.Duration()
            self._state_person_visible = msg.data
        else:
            self._state_duration_visible = rospy.Time.now() - self._state_last_visible

    def handler_lead_pose(self, goal):

        rospy.loginfo('New Goal: {}'.format(goal))

        # [m] [m] [rad]
        target_frame = goal.target_pose.header.frame_id
        target_x = goal.target_pose.pose.position.x
        target_y = goal.target_pose.pose.position.y
        target_z = goal.target_pose.pose.position.z
        target_qx = goal.target_pose.pose.orientation.x
        target_qy = goal.target_pose.pose.orientation.y
        target_qz = goal.target_pose.pose.orientation.z
        target_qw = goal.target_pose.pose.orientation.w

        #
        # calculate target pose at fixed frame
        #
        try:
            frame_fixed_to_target_frame = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self._frame_id_fixed,
                    target_frame,
                    rospy.Time(),
                    timeout=rospy.Duration(self._timeout_transform)
                )
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup transform failed. {}'.format(e))
            rospy.logerr('Aborted')
            self._action_server_lead_pos.set_aborted()
            return

        frame_target_frame_to_goal = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                target_qx,
                target_qy,
                target_qz,
                target_qw
            ),
            PyKDL.Vector(
                target_x,
                target_y,
                target_z
            )
        )

        frame_fixed_to_goal = frame_fixed_to_target_frame * frame_target_frame_to_goal

        #
        # Execute navigation
        #
        rate = rospy.Rate(1)
        state = 'none'  # trajectory, stop, or none
        trajectory_command_validtime = rospy.Time()
        while not rospy.is_shutdown():

            rate.sleep()

            if self._action_server_lead_pos.is_preempt_requested():
                rospy.loginfo('Preemptino requested.')
                self._action_server_lead_pos.set_preempted()
                return

            try:
                frame_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                    self._tf_buffer.lookup_transform(
                        self._frame_id_fixed,
                        self._frame_id_robot,
                        rospy.Time(),
                        timeout=rospy.Duration(self._timeout_transform)
                    )
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logerr('lookup transform failed. {}'.format(e))
                continue

            frame_robot_to_goal = frame_fixed_to_robot.Inverse() * frame_fixed_to_goal

            command_x = frame_robot_to_goal.p[0]
            command_y = frame_robot_to_goal.p[1]
            command_theta = frame_robot_to_goal.M.GetRPY()[2]

            rospy.loginfo('command_x: {}'.format(command_x))
            rospy.loginfo('command_y: {}'.format(command_y))
            rospy.loginfo('r p y: {}'.format(frame_robot_to_goal.M.GetRPY()))
            rospy.loginfo('command_theta: {}'.format(command_theta))
            # break if robot reach the goal
            if math.fabs(command_x) < self._tolerance_x \
                    and math.fabs(command_y) < self._tolerance_y \
                    and math.fabs(command_theta) < self._tolerance_theta:
                rospy.loginfo('Goal Reached.')
                self._action_server_lead_pos.set_succeeded(MoveBaseResult())
                break

            if state == 'trajectory':

                if not self._state_person_visible and self._state_duration_visible > rospy.Duration(1):
                    rospy.logwarn('No person visible')
                    self._spot_client.pubCmdVel(0, 0, 0)
                    state = 'stop'
                elif rospy.Time.now() > trajectory_command_validtime:
                    rospy.loginfo('Restart moving')
                    command_duration = 10
                    trajectory_command_validtime = rospy.Time.now() + rospy.Duration(command_duration - 2)
                    self._spot_client.trajectory(
                        command_x,
                        command_y,
                        command_theta,
                        command_duration,
                        blocking=False
                    )
                    state = 'trajectory'

            elif state == 'stop':

                if not self._state_person_visible and self._state_duration_visible > rospy.Duration(1):
                    rospy.logwarn('No person visible')
                    self._spot_client.pubCmdVel(0, 0, 0)
                    state = 'stop'
                else:
                    rospy.loginfo('Restart moving')
                    command_duration = 10
                    trajectory_command_validtime = rospy.Time.now() + rospy.Duration(command_duration - 2)
                    self._spot_client.trajectory(
                        command_x,
                        command_y,
                        command_theta,
                        command_duration,
                        blocking=False
                    )
                    state = 'trajectory'

            else:
                if not self._state_person_visible and self._state_duration_visible > rospy.Duration(1):
                    rospy.logwarn('No person visible')
                    self._spot_client.pubCmdVel(0, 0, 0)
                    state = 'stop'
                else:
                    rospy.loginfo('Start moving')
                    command_duration = 10
                    trajectory_command_validtime = rospy.Time.now() + rospy.Duration(command_duration - 2)
                    self._spot_client.trajectory(
                        command_x,
                        command_y,
                        command_theta,
                        command_duration,
                        blocking=False
                    )
                    state = 'trajectory'


if __name__ == '__main__':

    rospy.init_node('lead_pos_server')
    server = LeadPosServer()
    rospy.spin()
