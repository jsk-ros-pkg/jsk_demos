#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
import random
import threading

import actionlib
import rospy

from jsk_spot_delivery_demo.msg import DeliverToAction, DeliverToGoal
from jsk_spot_delivery_demo.msg import PickupPackageAction, PickupPackageGoal
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray

from spot_ros_client.libspotros import SpotRosClient
from sound_play.libsoundplay import SoundClient
from ros_speech_recognition import SpeechRecognitionClient

import smach
import smach_ros


def check_person():
    try:
        msg = rospy.wait_for_message('~person_visible', Bool,
                                     timeout=rospy.Duration(1))
        return msg.data
    except rospy.ROSException as e:
        rospy.logwarn('Timeout exceede: {}'.format(e))
        return False


def get_person_pose():
    try:
        msg = rospy.wait_for_message('~people_pose_array', PoseArray,
                                     timeout=rospy.Duration(1))
        if len(msg.poses) == 0:
            return None
        else:
            return random.choice(msg.poses), msg.header.frame_id
    except rospy.ROSException as e:
        rospy.logwarn('Timeout exceede: {}'.format(e))
        return None


class Ready(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_executing', 'strolling'])

    def execute(self, userdata):
        rospy.loginfo('Ready')

        if userdata.task_list.length() > 0 or userdata.task_executing is not None:
            return 'task_executing'

        return 'strolling'


class Strolling(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['approaching', 'strolling'])

    def execute(self, userdata):
        rospy.loginfo('Strolling')

        # Move to a node randomly selected.
        next_target = random.choice(userdata.list_node)
        rospy.loginfo('Moving to {}'.format(next_target))
        result = userdata.spot_ros_client.execute_behaviors(
            next_target, blocking=True)

        # Searching a person.
        timeout = rospy.Time.now() + rospy.Duration(120)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and rospy.Time.now() > timeout:
            rate.sleep()
            if check_person():
                return 'approaching'

        return 'strolling'


class Approaching(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['task_asking'])

    def execute(self, userdata):
        # get person pose
        pose, frame_id = get_person_pose()
        if pose is None:
            rospy.logwarn('No person')
            return 'ready'

        # approach
        pos = PyKDL.Vector(pose.position, x, pose.position.y, pose.position.z)
        pos = pos - 2.0 * pos / pos.Norm()
        x = pos[0]
        y = pos[1]
        theta = math.atan2(y, x)
        rospy.loginfo(
            'Found person at (x,y,theta) = ({},{},{})'.format(x, y, theta))
        userdata.spot_ros_client.trajectory(
            x, y, theta, rospy.Duration(10), blocking=True)

        return 'task_asking'


class TaskAsking(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ready'])
        self.actionclient_pickup_package = actionlib.SimpleActionClient(
            '~pickup_package', PickupPackageAction)

    def ask_task(self):

        self.actionclient_pickup_package.send_goal(
            PickupPackageGoal(timeout=rospy.Duration(60)))
        self.actionclient_pickup_package.wait_for_result()
        result = self.actionclient_pickup_package.get_result()
        return result.success, result.target_node_id

    def execute(self, userdata):
        rospy.loginfo('TaskAsking')

        # ask
        success, target_node_id = self.ask_task()
        if success:
            userdata.task_list.append(Task(target_node_id))

        return 'ready'


class TaskExecuting(smach.State):

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['ready'])
        self.actionclient_deliver_to = actionlib.SimpleActionClient(
            '~deliver_to', DeliverToAction)

    def execute(self, userdata):
        rospy.loginfo('TaskExecuting')
        if userdata.task_executing is not None and userdata.task_list.length() == 0:
            rospy.logwarn('No task left')
            return 'ready'
        elif userdata.task_executing is None and userdata.task_list.length() == 0:
            userdata.task_executing = userdata.pop(0)

        success = self.do_deliver_to(userdata.task_executing.target_node_id)
        if success:
            userdata.task_executing = None

        return 'ready'

    def do_deliver_to(self, target_node_id):

        goal = DeliverToGoal()
        goal.target_node_id = target_node_id
        goal.wait_package = True
        self.actionclient_deliver_to.send_goal(goal)
        self.actionclient_deliver_to.wait_for_result()
        result = self.actionclient_deliver_to.get_result()
        return result.success


class Task:

    def __init__(self, target_node_id):
        self.target_node_id = target_node_id
        self.num_trial = 0


class TaskList:

    def __init__(self):
        self.list = []
        self.lock = threading.Lock()

    def append(self, task):
        self.lock.acquire()
        self.list.append(task)
        self.lock.release()

    def pop(self):
        self.lock.acquire()
        task = self.list.pop(0)
        self.lock.release()
        return task

    def length(self):
        self.lock.acquire()
        length = len(self.list)
        self.lock.release()
        return length


def main():

    rospy.init_node('delivery_demo')

    sm = smach.StateMachine(outcomes=[''])
    sm.userdata.speech_recongition_client = SpeechRecognitionClient()
    sm.userdata.spot_ros_client = SpotRosClient()
    sm.userdata.sound_client = SoundClient(
        sound_action='/robotsound_jp', sound_topic='/robotsound_jp')
    sm.userdata.list_node_strolling = rospy.get_param(
        '~list_node_strolling', [])
    sm.userdata.task_list = TaskList()
    sm.userdata.task_executing = None

    with sm:
        smach.StateMachine.add('Ready', Ready(),
                               transitions={'task_executing':'TaskExecuting',
                                            'strolling':'Strolling'})
        smach.StateMachine.add('Strolling',Strolling(),
                               transitions={'approaching':'Approaching',
                                            'strolling':'Strolling'})
        smach.StateMachine.add('Approaching',Approaching(),
                               transitions={'task_asking':'TaskAsking'})
        smach.StateMachine.add('TaskAsking',Approaching(),
                               transitions={'ready':'Ready'})
        smach.StateMachine.add('TaskExecuting',TaskExecuting(),
                               transitions={'ready':'Ready'})


    rospy.loginfo('initialized')

    outcome = sm.execute()

    rospy.loginfo('outcome: {}'.format(outcome))


if __name__ == '__main__':
    main()
