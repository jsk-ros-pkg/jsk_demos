#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy
import message_filters
import PyKDL

import tf2_ros
import tf2_geometry_msgs

import math
import threading

from sound_play.libsoundplay import SoundClient

from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_msgs.msg import NavigateToAction, NavigateToGoal
from spot_msgs.srv import ListGraph, ListGraphRequest
from spot_msgs.srv import SetLocalizationFiducial, SetLocalizationFiducialRequest
from spot_msgs.srv import UploadGraph, UploadGraphRequest

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)



class Demo(object):

    def __init__(self):

        self._list_navigate_to = [
                # start point, end point, filepath
                [ 'start point', 'end point', 'filepath'  ],
                ]

        # rosservice call
        self._srv_list_graph = rospy.ServiceProxy('~list_graph', ListGraph)
        self._srv_set_localization_fiducial = rospy.ServiceProxy('~set_localization_fiducial', SetLocalizationFiducial)
        self._srv_upload_graph = rospy.ServiceProxy('~upload_graph', UploadGraph)

        # action client
        self._client_navigate_to = rospy.SimpleActionClient(
                                        '~navigate_to',
                                        NavigateToAction
                                        )

    def startNavigateTo(self,
                        start_point_name,
                        end_point_name):

        for navigate_to in self._list_navigate_to:
            if start_point_name == navigate_to[0] and end_point_name == navigate_to[1]:
                self.uploadGraph(navigate_to[2])
                waypoint_ids = self.listGraph()
                start_id = waypoint_ids[0]
                end_id = waypoint_ids[-1]
                goal = NavigateToGoal()
                goal.id_navigate_to = end_id
                return
            elif start_point_name == navigate_to[1] and end_point_name == navigate_to[0]:
                self.uploadGraph(navigate_to[2])
                start_id = waypoint_ids[-1]
                end_id = waypoint_ids[0]

    def uploadGraph(self, filepath):

        try:
            resp = self._srv_upload_graph(filepath)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Error: {}'.format(e))
            return False, 'Error: {}'.format(e)

    def setLocalizationFiducial(self):

        try:
            resp = self._srv_set_localization_fiducial()
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Error: {}'.format(e))
            return False, 'Error: {}'.format(e)

    def listGraph(self):

        try:
            resp = self._srv_list_graph()
            return resp
        except rospy.ServiceException as e:
            rospy.logerr('Error: {}'.format(e))
            return None

def main():

    rospy.init_node('follow_person_demo')
    demo = Demo()
    while not rospy.is_shutdown():

        rospy.sleep(1)
        with demo._lock_for_flag_server:
            if not demo._flag_server:
                continue
        target_id = demo.getNearestTargetID()
        if target_id is not None:
            demo.followTarget(target_id)
        else:
            rospy.logwarn('None target')
        with demo._lock_for_flag_server:
            demo._flag_server = False


if __name__ == '__main__':
    main()
