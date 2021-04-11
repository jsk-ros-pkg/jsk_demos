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

from spot_person_leader.msg import LeadPersonAction, LeadPersonFeedback, LeadPersonResult

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)

class LeadPersonDemo(object):

    '''
    LeadPersonDemo

    Subscriber:
        - '~bbox_array' (message type: jsk_recognition_msgs/BoundingBoxArray)

    Publisher:
        - '~cmd_vel' (message type: geometry_msgs/Twist)

    Service Client:
        - '~list_graph' (service type: spot_msgs/ListGraph)

        - '~set_localization_fiducial' (service type: spot_msgs/SetLocalizationFiducial)

        - '~upload_graph' (service type: spot_msgs/UploadGraph)

    Action Client:
        - '~navigate_to' (action type: spot_msgs/NavigateTo)

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

        # Publisher
        self._pub_cmd_vel = rospy.Publisher('~cmd_vel', Twist, queue_size=1)

        # rosservice call
        self._srv_list_graph = rospy.ServiceProxy('~list_graph', ListGraph)
        self._srv_set_localization_fiducial = rospy.ServiceProxy('~set_localization_fiducial', SetLocalizationFiducial)
        self._srv_upload_graph = rospy.ServiceProxy('~upload_graph', UploadGraph)

        # action client
        self._client_navigate_to = rospy.SimpleActionClient(
                                        '~navigate_to',
                                        NavigateToAction
                                        )

        # checking if person exists or not
        self._is_person_visible = False
        ## subscribe
        self._sub_bbox_array = rospy.Subscriber('~bbox_array',BoundingBoxArray,self._cb_bbox_array)

        # action server
        self._server_lead_person = rospy.SimpleActionServer(
                                        '~lead_person',
                                        LeadPersonAction,
                                        execute_cb=self._handler_lead_person,
                                        auto_start=False
                                        )
        self._server_lead_person.start()

    def _handler_lead_person(self, goal):

        if self.startNavigateTo( goal.start_point, goal.end_point ):
            rospy.loginfo('Navigate to started.')
        else:
            rospy.logerr('Route not found.')
            result = LeadPersonResult()
            result.success = False
            self._server_lead_person.set_aborted(result)
            return

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            rate.sleep()

            if self._server_lead_person.is_preempt_requested():
                rospy.logwarn('Action preempted.')
                # Cancel navigate to action
                self._client_navigate_to.cancel_goal()
                # Set aborted action server
                result = LeadPersonResult()
                result.success = False
                self._server_lead_person.set_preempted(result)
                return

            if self._client_navigate_to.wait_for_result(rospy.Duration(0.05)):
                break

            if not self._is_person_visible:
                self.publishZeroVelocity()

        result_navigate_to = self._client_navigate_to.get_result()
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
                    frame_id_robot,
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
                if vector_bbox_robotbased.Norm < self._dist_visible:
                    is_person_visible = True
                    break
        self._is_person_visible = is_person_visible

    def publishZeroVelocity(self):

        self._pub_cmd_vel(Twist())

    def startNavigateTo(self,
                        start_point,
                        end_point):

        for navigate_to in self._list_navigate_to:
            if start_point == navigate_to[0] and end_point == navigate_to[1]:
                self.uploadGraph(navigate_to[2])
                self.setLocalizationFiducial()
                waypoint_ids = self.listGraph()
                start_id = waypoint_ids[0]
                end_id = waypoint_ids[-1]
            elif start_point == navigate_to[1] and end_point == navigate_to[0]:
                self.uploadGraph(navigate_to[2])
                self.setLocalizationFiducial()
                start_id = waypoint_ids[-1]
                end_id = waypoint_ids[0]
            else:
                continue
            goal = NavigateToGoal()
            goal.id_navigate_to = end_id
            self._client_navigate_to.send_goal(goal)
            return True
        rospy.logerr('navigation route from {} to {} is not found.'.format(start_point, end_point))
        return False

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

    rospy.init_node('lead_person_demo')
    demo = LeadPersonDemo()
    rospy.spin()


if __name__ == '__main__':
    main()
