#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy
import PyKDL

import tf2_ros
import tf2_geometry_msgs

from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import Twist
from jsk_recognition_msgs.msg import BoundingBoxArray
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_msgs.msg import NavigateToAction, NavigateToGoal
from spot_msgs.srv import ListGraph, ListGraphRequest
from spot_msgs.srv import SetLocalizationFiducial, SetLocalizationFiducialRequest
from spot_msgs.srv import UploadGraph, UploadGraphRequest
from spot_person_leader.srv import GetStairRanges, GetStairRangesRequest

from spot_person_leader.msg import LeadPersonAction, LeadPersonFeedback, LeadPersonResult

from spot_ros_client.libspotros import SpotRosClient

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)

class LeadPersonDemo(object):

    '''
    LeadPersonDemo

    Subscriber:
        - '~bbox_array' (message type: jsk_recognition_msgs/BoundingBoxArray)

    Action Server:
        - '~lead_person' (action type: spot_msgs/LeadPerson)

    Parameters:
        - '~dist_visible' (float, default: 5.0)

        - '~duration_timeout' (float, default: 0.1)

        - '~label_person' (int, default: 0)

        - '~frame_id_robot' (str, default: 'body')

        - '~list_navigate_to' (list of list of str, default: [])
    '''

    def __init__(self):

        # navigation dictonary
        self._list_navigate_to = rospy.get_param('~list_navigate_to', [])

        # parameters
        self._dist_visible = float(rospy.get_param('~dist_visible',5.0))
        self._duration_timeout = float(rospy.get_param('~duration_timeout', 0.1))
        self._label_person = int(rospy.get_param('~label_person', 0))
        self._frame_id_robot = str(rospy.get_param('~frame_id_robot','body'))

        # rosservice call
        self._srv_get_stair_ranges = rospy.ServiceProxy('~get_stair_ranges', GetStairRanges)

        #
        self._spot_client = SpotRosClient();

        # tf
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # sound client
        self._sound_client = SoundClient(
                                    blocking=True,
                                    sound_action='/robotsound',
                                    sound_topic='/robotsound')

        # checking if person exists or not
        self._is_person_visible = False
        ## subscribe
        self._sub_bbox_array = rospy.Subscriber('~bbox_array',BoundingBoxArray,self._cb_bbox_array)

        # action server
        self._server_lead_person = actionlib.SimpleActionServer(
                                        '~lead_person',
                                        LeadPersonAction,
                                        execute_cb=self._handler_lead_person,
                                        auto_start=True
                                        )

        rospy.loginfo('Initialized!')
        rospy.loginfo('list_navigate_to: {}'.format(self._list_navigate_to))

    def _handler_lead_person(self, goal):

        rospy.loginfo('Lead Action started.')

        list_waypoint_id, list_ranges = self.settingupNavigateTo( goal.start_point, goal.end_point )

        rate = rospy.Rate(10)
        if list_waypoint_id is not None:
            self._sound_client.say('I will go to {}. Please follow me'.format(goal.end_point),blocking=True)
            for navigate_range in list_ranges:
                if navigate_range['is_stair']:
                    self._sound_client.say('Please go through the stairs and away from it.')
                    # TODO: wait until a person has gone up stairs
                    self._spot_client.navigate_to(list_waypoint_id[navigate_range['end_id']])
                    while True:
                        rate.sleep()
                        if rospy.is_shutdown():
                            rospy.logwarn('shutdown requested.')
                            # Cancel navigate to action
                            self._spot_client.cancel_navigate_to()
                            return
                        if self._server_lead_person.is_preempt_requested():
                            rospy.logwarn('Action preempted.')
                            # Cancel navigate to action
                            self._spot_client.cancel_navigate_to()
                            # Set aborted action server
                            result = LeadPersonResult()
                            result.success = False
                            self._server_lead_person.set_preempted(result)
                            return
                        if self._wait_for_navigate_to_result(rospy.Duration(0.05)):
                            rospy.loginfo('Navigate to action has finished')
                            break
                        if self._is_person_visible:
                            rospy.loginfo('Person is visible while stairs. Stopped.')
                            self._spot_client.pubCmdVel(0, 0, 0)
                else:
                    self._sound_client.say('Please follow me.')
                    self._spot_client.navigate_to(list_waypoint_id[navigate_range['end_id']])
                    while True:
                        rate.sleep()
                        if rospy.is_shutdown():
                            rospy.logwarn('shutdown requested.')
                            # Cancel navigate to action
                            self._spot_client.cancel_navigate_to()
                            return
                        if self._server_lead_person.is_preempt_requested():
                            rospy.logwarn('Action preempted.')
                            # Cancel navigate to action
                            self._spot_client.cancel_navigate_to()
                            # Set aborted action server
                            result = LeadPersonResult()
                            result.success = False
                            self._server_lead_person.set_preempted(result)
                            return
                        if self._wait_for_navigate_to_result(rospy.Duration(0.05)):
                            rospy.loginfo('Navigate to action has finished')
                            break
                        if not self._is_person_visible:
                            rospy.loginfo('No person is visible. Stopped.')
                            self._spot_client.pubCmdVel(0, 0, 0)
        else:
            rospy.logerr('Route not found.')
            result = LeadPersonResult()
            result.success = False
            self._server_lead_person.set_aborted(result)
            return

        self._sound_client.say('We have arrived at {}.'.format(goal.end_point))
        result_navigate_to = self._spot_client.get_navigate_to_result()
        result = LeadPersonResult()
        result.success = result_navigate_to.success
        self._server_lead_person.set_succeeded(result)

    def _cb_bbox_array(self,msg):

        time_observed = msg.header.stamp
        frame_id_msg = msg.header.frame_id
        frame_id_robot = self._frame_id_robot

        try:
            pykdl_transform_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    frame_id_robot,
                    frame_id_msg,
                    time_observed,
                    timeout=rospy.Duration(self._duration_timeout)
                )
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup transform failed. {}'.format(e))
            return

        # check if there is a person
        is_person_visible = False
        for bbox in msg.boxes:
            if bbox.label == self._label_person:
                vector_bbox = convert_msg_point_to_kdl_vector(bbox.pose.position)
                vector_bbox_robotbased = pykdl_transform_fixed_to_robot * vector_bbox
                if vector_bbox_robotbased.Norm() < self._dist_visible:
                    is_person_visible = True
                    break
        self._is_person_visible = is_person_visible

    def parseStairRanges(string):
        b = map(lambda x: x.split(','), string.split('),'))
        return map( lambda x: map( lambda y: int(y.replace(' ','').replace('[','').replace(']','').replace('(','').replace(')','')), x ), b )

    def settingupNavigateTo(self,
                        start_point,
                        end_point):

        for navigate_to in self._list_navigate_to:
            if start_point == navigate_to[0] and end_point == navigate_to[1]:
                self._spot_client.upload_graph(navigate_to[2])
                self._spot_client.set_localization_fiducial()
                stair_ranges = self.parseStairRanges(
                        self._srv_get_stair_ranges(GetStairRangesRequest(upload_filepath=navigate_to[2])).result)
                list_waypoint_id = self._spot_client.list_graph()
                list_ranges = []
                id_current = 0
                for stair_range in stair_ranges:
                    if id_current < stair_range[0]:
                        list_ranges.append(
                            {'start_id': id_current,
                             'end_id': stair_range[0],
                             'is_stair': false }
                            )
                    list_ranges.append(
                            {'start_id': stair_range[0],
                             'end_id': stair_range[1],
                             'is_stair': true }
                            )
                    id_current = stair_range[1]
                list_ranges.append(
                            {'start_id': id_cuurent,
                             'end_id': -1,
                             'is_stair': false }
                            )
                return list_waypoint_id, list_ranges
            else:
                continue
        rospy.logerr('navigation route from {} to {} is not found.'.format(start_point, end_point))
        return None, None


def main():

    rospy.init_node('lead_person_demo')
    demo = LeadPersonDemo()
    rospy.spin()


if __name__ == '__main__':
    main()
