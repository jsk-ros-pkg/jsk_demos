#!/usr/bin/env python
# -*- coding: utf-8 -*-

import networkx as nx

import actionlib
import rospy

from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient

from std_msgs.msg import String
from spot_person_leader.msg import LeadPersonAction, LeadPersonFeedback, LeadPersonResult
from spot_person_leader.srv import ResetCurrentNode, ResetCurrentNodeResponse
from std_msgs.msg import Bool



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
                               volume=10.0)

        result = LeadPersonResult(success=True)
        self._server_lead_person.set_succeeded(result)


    def navigate_edge(self, edge):

        if self._current_node != edge['from']:
            # Not valid
            return False

        if edge['type'] == 'walk':

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
                                   volume=10.0,
                                   blocking=True)

            success = False
            state_navigate = True
            rate = rospy.Rate(10)
            self._spot_client.navigate_to( end_id, blocking=False)
            while not rospy.is_shutdown():
                rate.sleep()

                if self._spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                    result = self._spot_client.get_navigate_to_result()
                    success = result.success
                    rospy.loginfo('result: {}'.format(result))
                    break

                if state_navigate and\
                        not self._state_visible and\
                        self._duration_visibility > rospy.Duration(5.0):
                    self._spot_client.cancel_navigate_to()
                    state_navigate = False
                elif not self._state_visible:
                    self._sound_client.say(
                            '近くに人が見えません',
                            volume=10.0,
                            blocking=True
                            )
                elif self._state_visible and not state_navigate:
                    self._spot_client.navigate_to( end_id, blocking=False)
                    state_navigate = True

            # recovery
            if not success:
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=10.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()

            return success

        elif edge['type'] == 'stair':

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

            self._sound_client.say(
                    '階段は危ないので私が昇り降りしている間は近づかないでください',
                    volume=10.0,
                    blocking=True)

            self._spot_client.navigate_to( end_id, blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

            # recovery
            if not result.success:
                self._sound_client.say(
                        '失敗したので元に戻ります',
                        volume=10.0,
                        blocking=True)
                self._spot_client.navigate_to( start_id, blocking=True)
                self._spot_client.wait_for_navigate_to_result()

            rate = rospy.Rate(10)
            self._sound_client.startWaveFromPkg('spot_person_leader','resources/akatonbo.ogg')
            while not rospy.is_shutdown():
                rate.sleep()
                if self._state_visible and self._duration_visibility > rospy.Duration(10):
                    self._sound_client.stopAll()
                    break

            self._sound_client.say(
                    'おまちしておりました',
                    volume=10.0,
                    blocking=True)

            return result.success

        elif edge['type'] == 'go_alone_and_wait':

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

            self._sound_client.say(
                    '私は階段で行くので、エレベーターで移動してください',
                    volume=10.0,
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
                        volume=10.0,
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
                        volume=10.0,
                        blocking=True)

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
