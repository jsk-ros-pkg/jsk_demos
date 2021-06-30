#!/usr/bin/env python
# -*- coding: utf-8 -*-

import networkx as nx

import threading

import actionlib
import rospy

from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from spot_person_leader.msg import LeadPersonAction, LeadPersonFeedback, LeadPersonResult
from spot_person_leader.srv import ResetCurrentNode, ResetCurrentNodeResponse
from std_msgs.msg import Bool
from switchbot_ros.msg import SwitchBotCommandGoal, SwitchBotCommandAction

import roslaunch
import rospkg

rospack = rospkg.RosPack()


class Map:

    def __init__(self, edges=[], nodes={}):

        self._edges = {}
        self._nodes = {}
        self._network = nx.DiGraph()

        self.loadGraph( edges, nodes )

    def loadGraph(self, edges, nodes):

        for key, node in nodes.items():
            self._nodes[key] = node

        for edge in edges:
            self._edges[edge['from'],edge['to']] = edge
            self._network.add_edge(edge['from'],edge['to'],weight=edge['cost'])

    def calcPath(self, node_from, node_to):

        node_list = nx.dijkstra_path( self._network, node_from, node_to )
        rospy.loginfo('node_list: {}'.format(node_list))
        path = []
        for index in range(len(node_list)-1):
            path.append(self._edges[node_list[index],node_list[index+1]])
        return path


class LeadPersonDemo(object):

    '''
    '''

    def __init__(self):

        # navigation dictonary
        edges = rospy.get_param('~map/edges')
        nodes = rospy.get_param('~map/nodes')
        self._map = Map(edges,nodes)
        self._current_node = rospy.get_param('~initial_node')
        self._pre_edge = None

        # parameter
        self._duration_visible_timeout = rospy.get_param('~duration_visible_timeout', 10.0)

        #
        self._spot_client = SpotRosClient();
        self._sound_client = SoundClient(
                                    blocking=False,
                                    sound_action='/robotsound_jp',
                                    sound_topic='/robotsound_jp'
                                    )

        # publisher
        self._pub_current_node = rospy.Publisher('~/current_node',String,queue_size=1)

        # reset service
        self._service_reset = rospy.Service(
                                    '~reset_current_node',
                                    ResetCurrentNode,
                                    self.handler_reset_current_node
                                    )

        #
        roslaunch.pmon._init_signal_handlers()

        # action server
        self._server_lead_person = actionlib.SimpleActionServer(
                                        '~lead_person',
                                        LeadPersonAction,
                                        execute_cb=self.handler_lead_person,
                                        auto_start=False
                                        )
        self._server_lead_person.start()

        rospy.loginfo('Initialized!')

    def spin(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            self._pub_current_node.publish(String(data=self._current_node))

    def handler_reset_current_node(self, req):

        self._current_node = req.current_node
        self._pre_edge = None

        return ResetCurrentNodeResponse(success=True)


    def handler_lead_person(self, goal):

        rospy.loginfo('Lead Action started.')

        path = self._map.calcPath( self._current_node, goal.target_node )

        self._sound_client.say('目的地に向かいます')

        for edge in path:
            try:
                if self.navigate_edge(edge):
                    rospy.loginfo('transition with edge {} succeeded'.format(edge))
                    self._current_node = edge['to']
                    self._pre_edge = edge
                else:
                    rospy.logerr('transition with edge {} failed'.format(edge))
                    result = LeadPersonResult(success=False)
                    self._sound_client.say('目的地に到達できませんでした'.format(goal.target_node),
                               volume=1.0)
                    self._server_lead_person.set_aborted(result)
                    return
            except Exception as e:
                rospy.logerr('Got an error with edge {}: {}'.format(edge, e))
                self._sound_client.say('エラーが発生しました'.format(goal.target_node),
                               volume=1.0)
                return

        self._sound_client.say('目的地に到着しました.'.format(goal.target_node),
                               volume=1.0)

        result = LeadPersonResult(success=True)
        self._server_lead_person.set_succeeded(result)


    def navigate_edge(self, edge):

        if self._current_node != edge['from']:
            # Not valid
            return False




        if edge['type'] == 'walk':
            #
            # Edge Type : walk
            #

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            detection_roslaunch_path = rospack.get_path('spot_person_leader') + '/launch/detections/walk_detection.launch'
            detection_roslaunch_cli_args = [detection_roslaunch_path]
            detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)
            detection_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                            uuid,
                                            detection_roslaunch_file
                                            )
            detection_roslaunch_parent.start()

            graph_name = edge['args']['graph']
            start_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['id']
            localization_method = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['localization_method']
            end_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['to']]['waypoints_on_graph']
                        )[0]['id']

            # register visibility subscriber
            self._tmp_state_visible = False
            self._tmp_starttime_visibility = rospy.Time.now()
            self._tmp_duration_visibility = rospy.Duration()

            def callback_visible(msg):

                if self._tmp_state_visible != msg.data:
                    self._tmp_starttime_visibility = rospy.Time.now()
                    self._tmp_duration_visibility = rospy.Duration()
                    self._tmp_state_visible = msg.data
                else:
                    self._tmp_duration_visibility = rospy.Time.now() - self._tmp_starttime_visibility

            subscriber_visible = rospy.Subscriber('/walk_detection_person_tracker/visible', Bool, callback_visible)
            rospy.loginfo('start subscription')

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                ret = self._spot_client.upload_graph(graph_name)
                if ret[0]:
                    rospy.loginfo('graph {} uploaded.'.format(graph_name))
                else:
                    rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False
                # Localization
                if localization_method == 'fiducial':
                    ret = self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    ret = self._spot_client.set_localization_waypoint(start_id)
                else:
                    ret = (False,'Unknown localization method')
                if ret[0]:
                    rospy.loginfo('robot is localized on the graph.')
                else:
                    rospy.logwarn('Localization failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False

            self._sound_client.say('ついてきてください',
                                   volume=1.0,
                                   blocking=True)

            success = False
            rate = rospy.Rate(10)
            self._spot_client.navigate_to( end_id, blocking=False)
            while not rospy.is_shutdown():
                rate.sleep()

                if self._spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                    result = self._spot_client.get_navigate_to_result()
                    success = result.success
                    rospy.loginfo('result: {}'.format(result))
                    break

                if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                    flag_speech = False
                    def notify_visibility():
                        self._sound_client.say(
                            '近くに人が見えません',
                            volume=1.0,
                            blocking=True
                            )
                        flag_speech = False
                    speech_thread = threading.Thread(target=notify_visibility)
                    speech_thread.start()
                    while not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                        rate.sleep()
                        self._spot_client.pubCmdVel(0,0,0)
                        if not flag_speech:
                            flag_speech = True
                            speech_thread = threading.Thread(target=notify_visibility)
                            speech_thread.start()
                        if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(5.0):
                            self._spot_client.cancel_navigate_to()
                        if self._tmp_state_visible:
                            self._spot_client.navigate_to( end_id, blocking=False)

            detection_roslaunch_parent.shutdown()
            subscriber_visible.unregister()
            del self._tmp_state_visible
            del self._tmp_starttime_visibility
            del self._tmp_duration_visibility

            # recovery
            if not success:
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=1.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()

            return success




        elif edge['type'] == 'narrow':
            #
            # Edge Type : narrow
            #

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            detection_roslaunch_path = rospack.get_path('spot_person_leader') + '/launch/detections/narrow_detection.launch'
            detection_roslaunch_cli_args = [detection_roslaunch_path]
            detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)
            detection_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                            uuid,
                                            detection_roslaunch_file,
                                            is_core=False
                                            )
            detection_roslaunch_parent.start()

            graph_name = edge['args']['graph']
            start_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['id']
            localization_method = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['localization_method']
            end_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['to']]['waypoints_on_graph']
                        )[0]['id']

            self._tmp_last_obstacle_right = rospy.Time()
            self._tmp_last_obstacle_left = rospy.Time()
            self._tmp_state_visible = False
            self._tmp_starttime_visibility = rospy.Time.now()
            self._tmp_duration_visibility = rospy.Duration()

            def obstacle_callback_right(msg):
                if len(msg.data) > 0:
                    self._tmp_last_obstacle_right = msg.header.stamp

            def obstacle_callback_left(msg):
                if len(msg.data) > 0:
                    self._tmp_last_obstacle_left = msg.header.stamp

            def callback_visible(msg):

                if self._tmp_state_visible != msg.data:
                    self._tmp_starttime_visibility = rospy.Time.now()
                    self._tmp_duration_visibility = rospy.Duration()
                    self._tmp_state_visible = msg.data
                else:
                    self._tmp_duration_visibility = rospy.Time.now() - self._tmp_starttime_visibility

            subscriber_obstacle_callback_right = rospy.Subscriber('/spot_recognition/right_obstacle', PointCloud2, obstacle_callback_right)
            subscriber_obstacle_callback_left = rospy.Subscriber('/spot_recognition/left_obstacle', PointCloud2, obstacle_callback_left)
            subscriber_visible = rospy.Subscriber('/narrow_detection_person_tracker/visible', Bool, callback_visible)

            rospy.loginfo('start subscription')

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                ret = self._spot_client.upload_graph(graph_name)
                if ret[0]:
                    rospy.loginfo('graph {} uploaded.'.format(graph_name))
                else:
                    rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_obstacle_callback_right.unregister()
                    subscriber_obstacle_callback_left.unregister()
                    subscriber_visible.unregister()
                    del self._tmp_last_obstacle_right
                    del self._tmp_last_obstacle_left
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False
                # Localization
                if localization_method == 'fiducial':
                    ret = self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    ret = self._spot_client.set_localization_waypoint(start_id)
                else:
                    ret = (False,'Unknown localization method')
                if ret[0]:
                    rospy.loginfo('robot is localized on the graph.')
                else:
                    rospy.logwarn('Localization failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_obstacle_callback_right.unregister()
                    subscriber_obstacle_callback_left.unregister()
                    subscriber_visible.unregister()
                    del self._tmp_last_obstacle_right
                    del self._tmp_last_obstacle_left
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False

            self._sound_client.say('ついてきてください',
                                   volume=1.0,
                                   blocking=True)

            flag_valid_obstacle_notification = True
            def notify_obstacle():
                rate = rospy.Rate(2)
                while flag_valid_obstacle_notification and not rospy.is_shutdown():
                    current_time = rospy.Time.now()
                    if current_time - self._tmp_last_obstacle_right < rospy.Duration(1.0) and current_time - self._tmp_last_obstacle_left < rospy.Duration(1.0):
                        self._sound_client.say(
                                '周囲にご注意ください',
                                blocking=True
                                )
                    elif current_time - self._tmp_last_obstacle_right < rospy.Duration(1.0):
                        self._sound_client.say(
                                '右にご注意ください',
                                blocking=True
                                )
                    elif current_time - self._tmp_last_obstacle_left < rospy.Duration(1.0):
                        self._sound_client.say(
                                '左にご注意ください',
                                blocking=True
                                )
                rospy.logwarn('notify_obstacle finished.')
            thread_notify_obstacle = threading.Thread(target=notify_obstacle)
            thread_notify_obstacle.start()

            success = False
            rate = rospy.Rate(10)
            self._spot_client.navigate_to( end_id, blocking=False)
            while not rospy.is_shutdown():
                rate.sleep()

                if self._spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                    result = self._spot_client.get_navigate_to_result()
                    success = result.success
                    rospy.loginfo('result: {}'.format(result))
                    break

                if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                    flag_speech = False
                    def notify_visibility():
                        self._sound_client.say(
                            '近くに人が見えません',
                            volume=1.0,
                            blocking=True
                            )
                        flag_speech = False
                    speech_thread = threading.Thread(target=notify_visibility)
                    speech_thread.start()
                    while not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                        rate.sleep()
                        self._spot_client.pubCmdVel(0,0,0)
                        if not flag_speech:
                            flag_speech = True
                            speech_thread = threading.Thread(target=notify_visibility)
                            speech_thread.start()
                        if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(5.0):
                            self._spot_client.cancel_navigate_to()
                        if self._tmp_state_visible:
                            self._spot_client.navigate_to( end_id, blocking=False)

            flag_valid_obstacle_notification = False
            thread_notify_obstacle.join()

            detection_roslaunch_parent.shutdown()
            subscriber_obstacle_callback_right.unregister()
            subscriber_obstacle_callback_left.unregister()
            subscriber_visible.unregister()
            del self._tmp_last_obstacle_right
            del self._tmp_last_obstacle_left
            del self._tmp_state_visible
            del self._tmp_starttime_visibility
            del self._tmp_duration_visibility

            # recovery
            if not success:
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=1.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()

            return success




        elif edge['type'] == 'crosswalk':
            #
            # Edge Type : crosswalk
            #

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            detection_roslaunch_path = rospack.get_path('spot_person_leader') + '/launch/detections/crosswalk_detection.launch'
            detection_roslaunch_cli_args = [detection_roslaunch_path]
            detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)
            detection_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                            uuid,
                                            detection_roslaunch_file
                                            )
            detection_roslaunch_parent.start()

            graph_name = edge['args']['graph']
            start_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['id']
            localization_method = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['localization_method']
            end_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['to']]['waypoints_on_graph']
                        )[0]['id']

            # register visibility subscriber
            self._tmp_state_visible = False
            self._tmp_starttime_visibility = rospy.Time.now()
            self._tmp_duration_visibility = rospy.Duration()

            def callback_visible(msg):

                if self._tmp_state_visible != msg.data:
                    self._tmp_starttime_visibility = rospy.Time.now()
                    self._tmp_duration_visibility = rospy.Duration()
                    self._tmp_state_visible = msg.data
                else:
                    self._tmp_duration_visibility = rospy.Time.now() - self._tmp_starttime_visibility

            subscriber_visible = rospy.Subscriber('/walk_detection_person_tracker/visible', Bool, callback_visible)
            rospy.loginfo('start subscription')

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                ret = self._spot_client.upload_graph(graph_name)
                if ret[0]:
                    rospy.loginfo('graph {} uploaded.'.format(graph_name))
                else:
                    rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False
                # Localization
                if localization_method == 'fiducial':
                    ret = self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    ret = self._spot_client.set_localization_waypoint(start_id)
                else:
                    ret = (False,'Unknown localization method')
                if ret[0]:
                    rospy.loginfo('robot is localized on the graph.')
                else:
                    rospy.logerr('Unknown localization method')
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False

            self._sound_client.say('車が通るかみています',
                                   volume=1.0,
                                   blocking=True)

            safety_count = 0
            while not rospy.is_shutdown():
                rospy.loginfo('safety_count: {}'.format(safety_count))
                if safety_count > 10:
                    break;
                try:
                    is_visible_car = rospy.wait_for_message('/crosswalk_detection_car_tracker/visible', Bool, rospy.Duration(1))
                    rospy.loginfo('is_visible_car: {}'.format(is_visible_car))
                    if is_visible_car == True:
                        safety_count = 0
                        self._sound_client.say('車が通ります',
                                   volume=1.0,
                                   blocking=True)

                    else:
                        safety_count += 1
                except Exception as e:
                    rospy.logwarn('{}'.format(e))
                    safety_count = 0

            self._sound_client.say('ついてきてください',
                                   volume=1.0,
                                   blocking=True)

            success = False
            self._spot_client.navigate_to( end_id, blocking=False)

            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                rate.sleep()

                if self._spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                    result = self._spot_client.get_navigate_to_result()
                    success = result.success
                    rospy.loginfo('result: {}'.format(result))
                    break

                if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                    flag_speech = False
                    def notify_visibility():
                        self._sound_client.say(
                            '近くに人が見えません',
                            volume=1.0,
                            blocking=True
                            )
                        flag_speech = False
                    speech_thread = threading.Thread(target=notify_visibility)
                    speech_thread.start()
                    while not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(0.5):
                        rate.sleep()
                        self._spot_client.pubCmdVel(0,0,0)
                        if not flag_speech:
                            flag_speech = True
                            speech_thread = threading.Thread(target=notify_visibility)
                            speech_thread.start()
                        if not self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(5.0):
                            self._spot_client.cancel_navigate_to()
                        if self._tmp_state_visible:
                            self._spot_client.navigate_to( end_id, blocking=False)

            detection_roslaunch_parent.shutdown()
            subscriber_visible.unregister()
            del self._tmp_state_visible
            del self._tmp_starttime_visibility
            del self._tmp_duration_visibility

            # recovery
            if not success:
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=1.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()

            return success




        elif edge['type'] == 'stair':
            #
            # Edge Type : stair
            #

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            detection_roslaunch_path = rospack.get_path('spot_person_leader') + '/launch/detections/stair_detection.launch'
            detection_roslaunch_cli_args = [detection_roslaunch_path]
            detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)
            detection_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                            uuid,
                                            detection_roslaunch_file
                                            )
            detection_roslaunch_parent.start()

            graph_name = edge['args']['graph']
            start_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['id']
            localization_method = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['localization_method']
            end_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['to']]['waypoints_on_graph']
                        )[0]['id']

            # register visibility subscriber
            self._tmp_state_visible = False
            self._tmp_starttime_visibility = rospy.Time.now()
            self._tmp_duration_visibility = rospy.Duration()

            def callback_visible(msg):

                if self._tmp_state_visible != msg.data:
                    self._tmp_starttime_visibility = rospy.Time.now()
                    self._tmp_duration_visibility = rospy.Duration()
                    self._tmp_state_visible = msg.data
                else:
                    self._tmp_duration_visibility = rospy.Time.now() - self._tmp_starttime_visibility

            subscriber_visible = rospy.Subscriber('/stair_detection_person_tracker/visible', Bool, callback_visible)
            rospy.loginfo('start subscription')

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                ret = self._spot_client.upload_graph(graph_name)
                if ret[0]:
                    rospy.loginfo('graph {} uploaded.'.format(graph_name))
                else:
                    rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False
                # Localization
                if localization_method == 'fiducial':
                    ret = self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    ret = self._spot_client.set_localization_waypoint(start_id)
                else:
                    ret = (False,'Unknown localization method')
                if ret[0]:
                    rospy.loginfo('robot is localized on the graph.')
                else:
                    rospy.logwarn('Localization failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False

            # check if there is a person lower than the robot.
            from geometry_msgs.msg import PoseArray
            while not rospy.is_shutdown():
                try:
                    people_pose_array = rospy.wait_for_message('/stair_detection_person_tracker/people_pose_array',PoseArray,rospy.Duration(10))
                except Exception as e:
                    rospy.logwarn('{}'.format(e))
                    continue
                exist_person_down = False
                for pose in people_pose_array.poses:
                    if pose.position.z < -0.5:
                        exist_person_down = True
                if exist_person_down:
                    self._sound_client.say('私より低い位置に誰かいるのでいなくなるまで待ちます')
                else:
                    break

            self._sound_client.say(
                    '階段は危ないので私が昇り降りしている間は近づかないでください',
                    volume=1.0,
                    blocking=True)

            self._spot_client.navigate_to( end_id, blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

            # recovery
            if not result.success:
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=1.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()
            else:
                rate = rospy.Rate(10)
                self._sound_client.startWaveFromPkg('spot_person_leader','resources/akatonbo.ogg')
                while not rospy.is_shutdown():
                    rate.sleep()
                    if self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(5):
                        self._sound_client.stopAll()
                        break

                self._sound_client.say(
                        'おまちしておりました',
                        volume=1.0,
                        blocking=True)

            detection_roslaunch_parent.shutdown()
            subscriber_visible.unregister()
            del self._tmp_state_visible
            del self._tmp_starttime_visibility
            del self._tmp_duration_visibility
            return result.success




        elif edge['type'] == 'go_alone_and_wait':
            #
            # Edge Type : go_alone_and_wait
            #

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            detection_roslaunch_path = rospack.get_path('spot_person_leader') + '/launch/detections/go_alone_and_wait_detection.launch'
            detection_roslaunch_cli_args = [detection_roslaunch_path]
            detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)
            detection_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                            uuid,
                                            detection_roslaunch_file
                                            )
            detection_roslaunch_parent.start()

            graph_name = edge['args']['graph']
            start_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['id']
            localization_method = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['localization_method']
            end_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['to']]['waypoints_on_graph']
                        )[0]['id']

            # register visibility subscriber
            self._tmp_state_visible = False
            self._tmp_starttime_visibility = rospy.Time.now()
            self._tmp_duration_visibility = rospy.Duration()

            def callback_visible(msg):

                if self._tmp_state_visible != msg.data:
                    self._tmp_starttime_visibility = rospy.Time.now()
                    self._tmp_duration_visibility = rospy.Duration()
                    self._tmp_state_visible = msg.data
                else:
                    self._tmp_duration_visibility = rospy.Time.now() - self._tmp_starttime_visibility

            subscriber_visible = rospy.Subscriber('/go_alone_and_wait_detection_person_tracker/visible', Bool, callback_visible)
            rospy.loginfo('start subscription')

            #
            action_client_switchbot = actionlib.SimpleActionClient(
                                            '/switchbot_ros/switch',
                                            SwitchBotCommandAction
                                            )
            if action_client_switchbot.wait_for_server(rospy.Duration(10)):
                #
                switchbot_goal = SwitchBotCommandGoal()
                switchbot_goal.device_name = self._map._nodes[edge['from']]['switchbot_device']
                switchbot_goal.command = 'press'
                action_client_switchbot.send_goal(switchbot_goal)

            self._sound_client.say(
                    '私は階段で行くので、エレベーターで{}階に移動してください'.format(
                        self._map._nodes[edge['to']]['floor']
                    ),
                    volume=1.0,
                    blocking=True)

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                ret = self._spot_client.upload_graph(graph_name)
                if ret[0]:
                    rospy.loginfo('graph {} uploaded.'.format(graph_name))
                else:
                    rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False
                # Localization
                if localization_method == 'fiducial':
                    ret = self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    ret = self._spot_client.set_localization_waypoint(start_id)
                else:
                    ret = (False,'Unknown localization method')
                if ret[0]:
                    rospy.loginfo('robot is localized on the graph.')
                else:
                    rospy.logwarn('Localization failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    subscriber_visible.unregister()
                    del self._tmp_state_visible
                    del self._tmp_starttime_visibility
                    del self._tmp_duration_visibility
                    return False

            self._spot_client.navigate_to( end_id, blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

            # recovery
            if not result.success:
                rospy.logwarn('失敗したので元に戻ります')
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=1.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()
            else:
                rate = rospy.Rate(10)
                self._sound_client.startWaveFromPkg('spot_person_leader','resources/akatonbo.ogg')
                while not rospy.is_shutdown():
                    rate.sleep()
                    if self._tmp_state_visible and self._tmp_duration_visibility > rospy.Duration(10):
                        self._sound_client.stopAll()
                        break
                self._sound_client.say(
                        'おまちしておりました',
                        volume=1.0,
                        blocking=True)

            detection_roslaunch_parent.shutdown()
            subscriber_visible.unregister()
            del self._tmp_state_visible
            del self._tmp_starttime_visibility
            del self._tmp_duration_visibility
            return result.success




        elif edge['type'] == 'elevator':
            #
            # Edge Type : elevator
            #

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            detection_roslaunch_path = rospack.get_path('spot_person_leader') + '/launch/detections/elevator_detection.launch'
            detection_roslaunch_cli_args = [detection_roslaunch_path]
            detection_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(detection_roslaunch_cli_args)
            detection_roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                            uuid,
                                            detection_roslaunch_file
                                            )
            detection_roslaunch_parent.start()

            graph_name = edge['args']['graph']
            start_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['id']
            localization_method = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['from']]['waypoints_on_graph']
                        )[0]['localization_method']
            rest_waypoint_id = edge['args']['rest_waypoint_id']
            end_id = filter(
                        lambda x: x['graph'] == graph_name,
                        self._map._nodes[edge['to']]['waypoints_on_graph']
                        )[0]['id']

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                ret = self._spot_client.upload_graph(graph_name)
                if ret[0]:
                    rospy.loginfo('graph {} uploaded.'.format(graph_name))
                else:
                    rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    return False
                # Localization
                if localization_method == 'fiducial':
                    ret = self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    ret = self._spot_client.set_localization_waypoint(start_id)
                else:
                    ret = (False,'Unknown localization method')
                if ret[0]:
                    rospy.loginfo('robot is localized on the graph.')
                else:
                    rospy.logwarn('Localization failed: {}'.format(ret[1]))
                    detection_roslaunch_parent.shutdown()
                    return False

            # Start checking if the door of a elevator is open
            self._tmp_door_is_open = False
            def door_point_callback(msg):
                if len(msg.data) == 0:
                    self._tmp_door_is_open = True
                else:
                    self._tmp_door_is_open = False

            subscriber_door_check = rospy.Subscriber(
                                        '/spot_recognition/elevator_door_outside_points',
                                        PointCloud2,
                                        door_point_callback)

            #
            action_client_switchbot = actionlib.SimpleActionClient(
                                            '/switchbot_ros/switch',
                                            SwitchBotCommandAction
                                            )
            if action_client_switchbot.wait_for_server(rospy.Duration(10)):
                #
                switchbot_goal = SwitchBotCommandGoal()
                switchbot_goal.device_name = self._map._nodes[edge['from']]['switchbot_device']
                switchbot_goal.command = 'press'
                action_client_switchbot.send_goal(switchbot_goal)
                action_client_switchbot.wait_for_result()
                result = action_client_switchbot.get_result()
                rospy.loginfo('switchbot result: {}'.format(result))

                if result.done:
                    self._sound_client.say(
                        'エレベーターで移動します',
                        volume=1.0,
                        blocking=True)
                else:
                    if self._map._nodes[edge['to']]['floor'] > self._map._nodes[edge['from']]['floor']:
                        self._sound_client.say(
                            'エレベーターで移動します. 上ボタンを押してください.',
                            volume=1.0,
                            blocking=True)
                    else:
                        self._sound_client.say(
                            'エレベーターで移動します. 下ボタンを押してください.',
                            volume=1.0,
                            blocking=True)
            else:
                rospy.logwarn('switchbot action client failed.')
                if self._map._nodes[edge['to']]['floor'] > self._map._nodes[edge['from']]['floor']:
                    self._sound_client.say(
                        'エレベーターで移動します. 上ボタンを押してください.',
                        volume=1.0,
                        blocking=True)
                else:
                    self._sound_client.say(
                        'エレベーターで移動します. 下ボタンを押してください.',
                        volume=1.0,
                        blocking=True)

            rate = rospy.Rate(2)
            while not rospy.is_shutdown():
                rate.sleep()
                if self._tmp_door_is_open:
                    break

            subscriber_door_check.unregister()
            self._sound_client.say('エレベーターが着きました', volume=1.0, blocking=False)

            self._spot_client.navigate_to( rest_waypoint_id, blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()
            ## recovery when riding on
            if not result.success:
                rospy.logwarn('失敗したので元に戻ります')
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=1.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()
                detection_roslaunch_parent.shutdown()
                return result.success
            else:
                self._sound_client.say(
                    '{}階を押してください'.format(
                        self._map._nodes[edge['to']]['floor']
                    ),
                    volume=1.0,
                    blocking=True)

            # check if the door of a elevator is open after once closed
            subscriber_door_check = rospy.Subscriber(
                                        '/spot_recognition/elevator_door_inside_points',
                                        PointCloud2,
                                        door_point_callback)
            rate = rospy.Rate(2)
            while not rospy.is_shutdown():
                rate.sleep()
                if not self._tmp_door_is_open:
                    break
            rospy.loginfo('door closed')
            while not rospy.is_shutdown():
                rate.sleep()
                if self._tmp_door_is_open:
                    break
            rospy.loginfo('door opened')

            self._sound_client.say(
                    '{}階に着きました'.format(
                        self._map._nodes[edge['to']]['floor']
                        ),
                    volume=1.0,
                    blocking=False)
            self._spot_client.navigate_to( end_id, blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

            detection_roslaunch_parent.shutdown()
            return result.success




        else:
            rospy.logerr('Unknown edge type.')
            return False


def main():

    rospy.init_node('lead_person_demo')
    demo = LeadPersonDemo()
    rospy.spin()


if __name__ == '__main__':
    main()
