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

        node_list = nx.shortest_path( self._network, node_from, node_to )
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

        # action client
        self._ac_switchbot = actionlib.SimpleActionClient(
                                    '/switchbot_ros/switch',
                                    SwitchBotCommandAction
                                    )

        # reset service
        self._service_reset = rospy.Service(
                                    '~reset_current_node',
                                    ResetCurrentNode,
                                    self.handler_reset_current_node
                                    )

        # subscriber
        self._state_visible = False
        self._starttime_visibility = rospy.Time.now()
        self._duration_visibility = rospy.Duration()
        self._sub_visible = rospy.Subscriber(
                                    '~visible',
                                    Bool,
                                    self.callback_visible
                                    )

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

    def callback_visible(self, msg):

        if self._state_visible != msg.data:
            self._starttime_visibility = rospy.Time.now()
            self._duration_visibility = rospy.Duration()
            self._state_visible = msg.data
        else:
            self._duration_visibility = rospy.Time.now() - self._starttime_visibility


    def handler_reset_current_node(self, req):

        self._current_node = req.current_node
        self._pre_edge = None

        return ResetCurrentNodeResponse(success=True)


    def handler_lead_person(self, goal):

        rospy.loginfo('Lead Action started.')

        path = self._map.calcPath( self._current_node, goal.target_node )

        self._sound_client.say('目的地に向かいます')

        for edge in path:
            if self.navigate_edge(edge):
                rospy.loginfo('transition with edge {} succeeded'.format(edge))
                self._current_node = edge['to']
                self._pre_edge = edge
            else:
                rospy.logerr('transition with edge {} failed'.format(edge))
                result = LeadPersonResult(success=False)
                self._server_lead_person.set_aborted(result)
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

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                self._spot_client.upload_graph( graph_name )
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
                # Localization
                if localization_method == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    self._spot_client.set_localization_waypoint(start_id)
                else:
                    rospy.logerr('Unknown localization method')
                    return False
                rospy.loginfo('robot is localized on the graph.')

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

                if not self._state_visible and self._duration_visibility > rospy.Duration(0.5):
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
                    while not self._state_visible and self._duration_visibility > rospy.Duration(0.5):
                        rate.sleep()
                        self._spot_client.pubCmdVel(0,0,0)
                        if not flag_speech:
                            flag_speech = True
                            speech_thread = threading.Thread(target=notify_visibility)
                            speech_thread.start()
                        if not self._state_visible and self._duration_visibility > rospy.Duration(5.0):
                            self._spot_client.cancel_navigate_to()
                        if self._state_visible:
                            self._spot_client.navigate_to( end_id, blocking=False)

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

            def obstacle_callback_right(msg):
                if len(msg.data) > 0:
                    self._tmp_last_obstacle_right = msg.header.stamp

            def obstacle_callback_left(msg):
                if len(msg.data) > 0:
                    self._tmp_last_obstacle_left = msg.header.stamp

            subscriber_obstacle_callback_right = rospy.Subscriber('/spot_recognition/right_obstacle', PointCloud2, obstacle_callback_right)
            subscriber_obstacle_callback_left = rospy.Subscriber('/spot_recognition/left_obstacle', PointCloud2, obstacle_callback_left)

            rospy.loginfo('start obstacle subscription')

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                self._spot_client.upload_graph( graph_name )
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
                # Localization
                if localization_method == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    self._spot_client.set_localization_waypoint(start_id)
                else:
                    rospy.logerr('Unknown localization method')
                    return False
                rospy.loginfo('robot is localized on the graph.')


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

                if not self._state_visible and self._duration_visibility > rospy.Duration(0.5):
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
                    while not self._state_visible and self._duration_visibility > rospy.Duration(0.5):
                        rate.sleep()
                        self._spot_client.pubCmdVel(0,0,0)
                        if not flag_speech:
                            flag_speech = True
                            speech_thread = threading.Thread(target=notify_visibility)
                            speech_thread.start()
                        if not self._state_visible and self._duration_visibility > rospy.Duration(5.0):
                            self._spot_client.cancel_navigate_to()
                        if self._state_visible:
                            self._spot_client.navigate_to( end_id, blocking=False)

            flag_valid_obstacle_notification = False
            thread_notify_obstacle.join()
            subscriber_obstacle_callback_right.unregister()
            subscriber_obstacle_callback_left.unregister()
            del self._tmp_last_obstacle_right
            del self._tmp_last_obstacle_left

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

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                self._spot_client.upload_graph( graph_name )
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
                # Localization
                if localization_method == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    self._spot_client.set_localization_waypoint(start_id)
                else:
                    rospy.logerr('Unknown localization method')
                    return False
                rospy.loginfo('robot is localized on the graph.')

            safety_count = 0
            while not rospy.is_shutdown():
                if safety_count > 10:
                    break;
                try:
                    is_visible_car = rospy.wait_for_message('/car_tracker/visible', Bool)
                    if is_visible_car:
                        safety_count = 0
                    else:
                        safety_count += 1
                except Exception as e:
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

                if not self._state_visible and self._duration_visibility > rospy.Duration(0.5):
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
                    while not self._state_visible and self._duration_visibility > rospy.Duration(0.5):
                        rate.sleep()
                        self._spot_client.pubCmdVel(0,0,0)
                        if not flag_speech:
                            flag_speech = True
                            speech_thread = threading.Thread(target=notify_visibility)
                            speech_thread.start()
                        if not self._state_visible and self._duration_visibility > rospy.Duration(5.0):
                            self._spot_client.cancel_navigate_to()
                        if self._state_visible:
                            self._spot_client.navigate_to( end_id, blocking=False)

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

            # graph uploading and localization
            if self._pre_edge is not None and \
                graph_name == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                # Upload
                self._spot_client.upload_graph( graph_name )
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
                # Localization
                if localization_method == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    self._spot_client.set_localization_waypoint(start_id)
                else:
                    rospy.logerr('Unknown localization method')
                    return False
                rospy.loginfo('robot is localized on the graph.')

            # check if there is a person lower than the robot.
            from geometry_msgs.msg import PoseArray
            while not rospy.is_shutdown():
                people_pose_array = rospy.wait_for_message('/person_tracker/people_pose_array',PoseArray)
                exist_person_down = False
                for pose in PoseArray.poses:
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
                    if self._state_visible and self._duration_visibility > rospy.Duration(5):
                        self._sound_client.stopAll()
                        break

                self._sound_client.say(
                        'おまちしておりました',
                        volume=1.0,
                        blocking=True)

            return result.success




        elif edge['type'] == 'go_alone_and_wait':
            #
            # Edge Type : go_alone_and_wait
            #

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

            switchbot_goal = SwitchBotCommandGoal()
            switchbot_goal.device_name = self._map._nodes[edge['from']]['switchbot_device']
            switchbot_goal.command = 'press'
            self._ac_switchbot.send_goal(switchbot_goal)

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
                self._spot_client.upload_graph( graph_name )
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
                # Localization
                if localization_method == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    self._spot_client.set_localization_waypoint(start_id)
                else:
                    rospy.logerr('Unknown localization method')
                    return False
                rospy.loginfo('robot is localized on the graph.')

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
                    if self._state_visible and self._duration_visibility > rospy.Duration(10):
                        self._sound_client.stopAll()
                        break
                self._sound_client.say(
                        'おまちしておりました',
                        volume=1.0,
                        blocking=True)

            return result.success




        elif edge['type'] == 'elevator':
            #
            # Edge Type : elevator
            #

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
                self._spot_client.upload_graph( graph_name )
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
                # Localization
                if localization_method == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                elif localization_method == 'waypoint':
                    self._spot_client.set_localization_waypoint(start_id)
                else:
                    rospy.logerr('Unknown localization method')
                    return False
                rospy.loginfo('robot is localized on the graph.')

            switchbot_goal = SwitchBotCommandGoal()
            switchbot_goal.device_name = self._map._nodes[edge['from']]['switchbot_device']
            switchbot_goal.command = 'press'
            self._ac_switchbot.send_goal(switchbot_goal)
            self._ac_switchbot.wait_for_result()
            result = self._ac_switchbot.get_result()
            rospy.loginfo('switchbot result: {}'.format(result))

            if result.done:
                self._sound_client.say(
                    'エレベーターで移動します',
                    volume=1.0,
                    blocking=True)
            else:
                if self._map._nodes[edge['to']]['floor'] > self._map._nodes[edge['to']]['floor']:
                    self._sound_client.say(
                        'エレベーターで移動します. 上ボタンを押してください.',
                        volume=1.0,
                        blocking=True)
                else:
                    self._sound_client.say(
                        'エレベーターで移動します. 下ボタンを押してください.',
                        volume=1.0,
                        blocking=True)

            # TODO: check if the door of a elevator is open
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
                return result.success
            else:
                self._sound_client.say(
                    '{}階を押してください'.format(
                        self._map._nodes[edge['to']]['floor']
                    ),
                    volume=1.0,
                    blocking=True)

            # TODO: check if the door of a elevator is open after once closed
            self._sound_client.say(
                    '20秒まちます',
                    volume=1.0,
                    blocking=True)
            rospy.sleep(20)

            self._spot_client.navigate_to( end_id, blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

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
