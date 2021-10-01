#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math

import actionlib
import rospy

from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import WrenchStamped
from jsk_spot_delivery_demo.msg import DeliverToAction, DeliverToResult
from jsk_spot_delivery_demo.msg import PickupPackageAction, PickupPackageResult
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal

from spot_ros_client.libspotros import SpotRosClient
from sound_play.libsoundplay import SoundClient
from ros_speech_recognition import SpeechRecognitionClient


class DeliveryActionServer:

    def __init__(self):

        self.srvclient_reset_body_force = rospy.ServiceProxy(
            '~reset_body_force', Empty)

        self.speech_recognition_client = SpeechRecognitionClient()
        self.spot_ros_client = SpotRosClient()
        self.sound_client = SoundClient(
            sound_action='/robotsound_jp', sound_topic='/robotsound_jp')

        self.node_list = rospy.get_param('~nodes', {})

        self.actionserver_deliver_to = actionlib.SimpleActionServer(
            '~deliver_to', DeliverToAction, self.callback_deliver_to)
        self.actionserver_pickup_pacakge = actionlib.SimpleActionServer(
            '~pickup_pacakge', PickupPackageAction, self.callback_pickup_pacakge)

    def wait_package_setting(self, duration=rospy.Duration(120)):

        self.done_pick_or_place = False
        force_threshold = rospy.get_param('~force_threshold', 5)
        self.srvclient_reset_body_force()

        def callback(msg):
            rospy.logdebug('force z: {}, threshold: {}'.format(
                math.fabs(msg.wrench.force.z), force_threshold))
            if math.fabs(msg.wrench.force.z) > force_threshold:
                self.done_pick_or_place = True
        sub = rospy.Subscriber('~body_force', WrenchStamped, callback)
        rate = rospy.Rate(10)
        timeout_deadline = rospy.Time.now() + duration
        while not rospy.is_shutdown():
            rate.sleep()
            if self.done_pick_or_place or rospy.Time.now() < timeout_deadline:
                break
        success = self.done_pick_or_place
        sub.unregister()
        del self.done_pick_or_place
        return success

    def callback_pickup_package(self, goal):

        timeout_deadline = rospy.Time.now() + goal.timeout

        rospy.loginfo('Asking package information')
        success = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_deadline:
            self.sound_client.say('配達先を教えてください。', blocking=True)
            recogntion_result = self.speech_recognition_client.recognize()
            target_node_candidates = {}
            for node_id, value in self.node_list.values():
                if 'name_jp' in value and value['name_jp'] in recogntion_result:
                    target_node_candidates[node_id] = value
            if len(target_node_candidates) == 0:
                rospy.logerr(
                    'No matching node found from spoken \'{}\''.format(recogntion_result))
                self.sound_client.say('配達先がわかりませんでした', blocking=True)
            else:
                success = True
                break

        if not success:
            self.actionserver_deliver_to.set_aborted(
                DeliverToResult(False, 'Falied to recognize the destination from speech.'))
            return
        else:
            target_node_id = target_node_candidates.keys()[0]
            target_node_name_jp = self.node_list[target_node_id]['name_jp']
            rospy.loginfo('target_node_id: {}'.format(target_node_id))
            self.sound_client.say(
                '{}ですね。わかりました。'.format(target_node_name_jp))

        rospy.loginfo('Waiting for package placed.')
        self.sound_client.say('荷物を置いてください', blocking=True)
        success = self.wait_package_setting(
            timeout_deadline - rospy.Time.now())
        if success:
            rospy.loginfo('Package placed')
            self.sound_client.say('荷物を確認しました', blocking=True)
            self.actionserver_pickup_pacakge.set_succeeded(
                PickupPackageResult(True, target_node_id)
            )
        else:
            rospy.logwerr('Timeout')
            self.actionserver_deliver_to.set_aborted(
                DeliverToResult(False, 'Falied to recognize package.'))

    def callback_deliver_to(self, goal):

        target_node_id = goal.target_node_id

        rospy.loginfo('move to {}'.format(target_node_id))
        result = self.spot_ros_client.execute_behaviors(target_node_id)
        if not result:
            rospy.logerr('Failed to reach {}'.format(target_node_id))
            self.actionserver_deliver_to.set_aborted(
                DeliverToResult(False, 'Falied to reach the destination.'))
            return
        rospy.loginfo('reached {}'.format(target_node_id))

        while not rospy.is_shutdown():
            rospy.loginfo('Waiting for package picked.')
            self.sound_client.say('荷物を受け取ってください', blocking=True)
            if self.wait_package_setting(rospy.Duration(5)):
                rospy.loginfo('Package picked')
                break

        self.actionserver_deliver_to.set_succeeded(
            DeliverToResult(True, 'Success'))
        return


def main():

    rospy.init_node('delivery_action_server')
    node = DeliveryServer()
    rospy.spin()


if __name__ == '__main__':
    main()
