#!/usr/bin/env python
# -*- coding: utf-8 -*-

import networkx as nx

import actionlib
import rospy
import PyKDL

import tf2_ros
import tf2_geometry_msgs

from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient

from jsk_recognition_msgs.msg import BoundingBoxArray
from spot_person_leader.srv import GetStairRanges, GetStairRangesRequest

from spot_person_leader.msg import LeadPersonAction, LeadPersonFeedback, LeadPersonResult



def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)


class Map:

    def __init__(self, edges=[], nodes=[]):

        self._edges = {}
        self._nodes = {}
        self._network = nx.DiGraph()

        self.loadGraph( edges, nodes )

    def loadGraph(self, edges, nodes):

        for node in nodes:
            self._nodes[node['name']] = node

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

        # tf
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)


        #
        self._spot_client = SpotRosClient();
        self._sound_client = SoundClient(
                                    blocking=False,
                                    sound_action='/robotsound_jp',
                                    sound_topic='/robotsound_jp'
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

    def handler_lead_person(self, goal):

        rospy.loginfo('Lead Action started.')

        path = self._map.calcPath( self._current_node, goal.target_node )

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

        self._sound_client.say('目的地 {} に到着しました.'.format(goal.target_node))

        result = LeadPersonResult(success=True)
        self._server_lead_person.set_succeeded(result)

    def navigate_edge(self, edge):

        if self._current_node != edge['from']:
            # Not valid
            return False

        if edge['type'] == 'walk':

            # graph uploading and localization
            if self._pre_edge is not None and \
                edge['args']['graph'] == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                self._spot_client.upload_graph( edge['args']['graph'] )
                rospy.loginfo('graph {} uploaded.'.format(edge['args']['graph']))
                if edge['args']['localization_method'] == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                else:
                    # Not implemented
                    rospy.logerr('localization_method other than fiducial is not implemented yet.')
                    return False
                rospy.loginfo('robot is localized on the graph.')

            self._sound_client.say('ついてきてください', blocking=True)

            self._spot_client.navigate_to(edge['args']['end_id'], blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

            return result.success

        elif edge['type'] == 'stair':

            # graph uploading and localization
            if self._pre_edge is not None and \
                edge['args']['graph'] == self._pre_edge['args']['graph']:
                rospy.loginfo('graph upload and localization skipped.')
            else:
                self._spot_client.upload_graph( edge['args']['graph'] )
                rospy.loginfo('graph {} uploaded.'.format(edge['args']['graph']))
                if edge['args']['localization_method'] == 'fiducial':
                    self._spot_client.set_localization_fiducial()
                else:
                    # Not implemented
                    rospy.logerr('localization_method other than fiducial is not implemented yet.')
                    return False
                rospy.loginfo('robot is localized on the graph.')

            self._sound_client.say('階段は危ないので先に行ってください', blocking=True)

            self._spot_client.navigate_to(edge['args']['end_id'], blocking=True)
            self._spot_client.wait_for_navigate_to_result()
            result = self._spot_client.get_navigate_to_result()

            self._sound_client.say('おまたせしました', blocking=True)

            return result.success

        else:
            # Unknown edge type
            return False


def main():

    rospy.init_node('lead_person_demo')
    demo = LeadPersonDemo()
    rospy.spin()


if __name__ == '__main__':
    main()
