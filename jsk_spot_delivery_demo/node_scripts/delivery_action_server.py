#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math

import actionlib
import rospy
import PyKDL

from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import WrenchStamped, Quaternion, PoseArray, PoseStamped
from jsk_spot_delivery_demo.msg import DeliverToAction, DeliverToResult
from jsk_spot_delivery_demo.msg import PickupPackageAction, PickupPackageResult
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal

from spot_ros_client.libspotros import SpotRosClient
from sound_play.libsoundplay import SoundClient
from ros_speech_recognition import SpeechRecognitionClient


def calc_distance(pose):

    return pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)


def get_nearest_person_pose():

    try:
        msg = rospy.wait_for_message('~people_pose_array', PoseArray,
                                     timeout=rospy.Duration(5))
    except rospy.ROSException as e:
        rospy.logwarn('Timeout exceede: {}'.format(e))
        return None

    if len(msg.poses) == 0:
        rospy.logwarn('No person visible')
        return None

    distance = calc_distance(msg.poses[0])
    target_pose = msg.poses[0]
    for pose in msg.poses:
        if calc_distance(pose) < distance:
            distance = calc_distance(pose)
            target_pose = pose

    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = target_pose

    return pose_stamped


def get_diff_for_person(pose_stamped):

    vector_person_msgbased = convert_msg_point_to_kdl_vector(pose_stamped.pose.position)
    x = pose_stamped.pose.position.x
    y = pose_stamped.pose.position.y
    z = pose_stamped.pose.position.z

    yaw = math.atan2(y,x)
    pitch = math.acos( z / math.sqrt(x**2 + y**2))
    return pitch, yaw


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
            '~deliver_to', DeliverToAction, self.callback_deliver_to, auto_start=False)
        self.actionserver_pickup_package = actionlib.SimpleActionServer(
            '~pickup_package', PickupPackageAction, self.callback_pickup_package, auto_start=False)

        self.actionserver_deliver_to.start()
        self.actionserver_pickup_package.start()

        rospy.loginfo('initialized')

    def head_for_person(self, use_pitch=True):

        self.spot_ros_client.pubBodyPose(0,Quaternion(x=0,y=0,z=0,w=1))
        pose = get_nearest_person_pose()
        if pose is None:
            return False
        pitch, yaw = get_diff_for_person(pose)
        self.spot_ros_client.trajectory(0,0,yaw,5,blocking=True)
        if use_pitch:
            self.spot_ros_client.pubBodyPose(0,Quaternion(x=0,y=math.sin(-pitch/2),z=0,w=math.cos(-pitch/2)))
        return True

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
            if self.done_pick_or_place or rospy.Time.now() > timeout_deadline:
                break
        success = self.done_pick_or_place
        sub.unregister()
        del self.done_pick_or_place
        return success

    def callback_pickup_package(self, goal):

        result = PickupPackageResult()
        timeout_deadline = rospy.Time.now() + goal.timeout

        rospy.loginfo('Asking package information')
        success = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_deadline:
            self.head_for_person()
            self.sound_client.say('配達先を教えてください。', blocking=True)
            recognition_result = self.speech_recognition_client.recognize()
            if len(recognition_result.transcript) == 0:
                rospy.logerr('No matching node found from spoken \'{}\''.format(recognition_result))
                self.sound_client.say('配達先がわかりませんでした', blocking=True)
                continue
            recognized_destination = recognition_result.transcript[0]
            target_node_candidates = {}
            for node_id, value in self.node_list.items():
                try:
                    if not value.has_key('name_jp'):
                        continue
                    if type(value['name_jp']) is list:
                        # DO HOGE
                        for name in value['name_jp']:
                            if name.encode('utf-8') == recognized_destination:
                                target_node_candidates[node_id] = value
                    else:
                        if value['name_jp'].encode('utf-8') == recognized_destination:
                            target_node_candidates[node_id] = value
                except Exception as e:
                    rospy.logerr('Error: {}'.format(e))
            if len(target_node_candidates) == 0:
                rospy.logerr('No matching node found from spoken \'{}\''.format(recognition_result))
                self.sound_client.say('配達先がわかりませんでした', blocking=True)
            else:
                success = True
                break

        if not success:
            result.success = False
            result.message = 'Falied to recognize the destination from speech.'
            self.actionserver_pickup_package.set_aborted(result)
            return
        else:
            target_node_id = target_node_candidates.keys()[0]
            target_node_name_jp = self.node_list[target_node_id]['name_jp'].encode('utf-8')
            rospy.loginfo('target_node_id: {}'.format(target_node_id))
            self.sound_client.say('{}ですね。わかりました。'.format(target_node_name_jp),blocking=True)


        rospy.loginfo('Asking sender information')
        success = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_deadline:
            self.head_for_person()
            self.sound_client.say('送り主の名前を教えてください', blocking=True)
            recognition_result = self.speech_recognition_client.recognize()
            if len(recognition_result.transcript) == 0:
                rospy.logerr('No spoken result: \'{}\''.format(recognition_result))
                self.sound_client.say('聞き取れませんでした', blocking=True)
                continue
            recognized_name = recognition_result.transcript[0]
            self.sound_client.say('{}さんですね',blocking=True)
            success = True

        if not success:
            result.success = False
            result.message = 'Falied to recognize sender name from speech.'
            self.actionserver_pickup_package.set_aborted(result)
            return


        rospy.loginfo('Waiting for package placed.')
        self.head_for_person(use_pitch=False)
        self.sound_client.say('荷物を置いてください', blocking=True)
        success = self.wait_package_setting(timeout_deadline - rospy.Time.now())
        if not success:
            rospy.logerr('Timeout for package placement')
            result.success = False
            result.message = 'Timeout for package placement.'
            self.actionserver_pickup_package.set_aborted(result)
        else:
            rospy.loginfo('Package placed')
            self.sound_client.say('荷物を確認しました', blocking=True)


        result.success = True
        result.message = 'Success'
        result.task.target_node_id = target_node_id
        result.task.package_content = ''
        result.task.sender = sender_name
        self.actionserver_pickup_package.set_succeeded(result)


    def callback_deliver_to(self, goal):

        task = goal.task
        target_node_id = task.target_node_id

        rospy.loginfo('move to {}'.format(task.target_node_id))
        result = self.spot_ros_client.execute_behaviors(task.target_node_id)
        rospy.logwarn('result: {}'.format(result))
        if not result.success:
            rospy.logerr('Failed to reach {}'.format(task.target_node_id))
            self.actionserver_deliver_to.set_aborted(
                DeliverToResult(False, 'Falied to reach the destination.'))
            return
        rospy.loginfo('reached {}'.format(task.target_node_id))

        while not rospy.is_shutdown():
            rospy.loginfo('Waiting for package picked.')
            self.sound_client.say(
                    '{}さんから{}のお届けです、荷物を受け取ってください'.format(task.sender_name,task.package_content),
                    blocking=True)
            if self.wait_package_setting(rospy.Duration(5)):
                rospy.loginfo('Package picked')
                break

        self.actionserver_deliver_to.set_succeeded(
            DeliverToResult(True, 'Success'))
        return


def main():

    rospy.init_node('delivery_action_server')
    node = DeliveryActionServer()
    rospy.spin()


if __name__ == '__main__':
    main()
