#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
import random
import threading

import actionlib
import rospy
import PyKDL

from jsk_spot_delivery_demo.msg import DeliverToAction, DeliverToGoal
from jsk_spot_delivery_demo.msg import PickupPackageAction, PickupPackageGoal
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, PoseStamped

from spot_ros_client.libspotros import SpotRosClient
from sound_play.libsoundplay import SoundClient
from ros_speech_recognition import SpeechRecognitionClient

import smach
import smach_ros

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


class Task:

    def __init__(self, target_node_id, content, sender):
        self.target_node_id = target_node_id
        self.content = content
        self.sender = sender
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


data_speech_recongition_client = None
data_spot_ros_client = None
data_sound_client = None
data_list_node_strolling = None
data_task_list_= None
data_task_executing = None

def main():

    rospy.init_node('delivery_demo')

    sm = smach.StateMachine(outcomes=[''])

    global data_speech_recongition_client
    global data_spot_ros_client
    global data_sound_client
    global data_list_node_strolling
    global data_task_list
    global data_task_executing

    # read only
    data_speech_recongition_client = SpeechRecognitionClient()
    data_spot_ros_client = SpotRosClient()
    data_sound_client = SoundClient(sound_action='/robotsound_jp', sound_topic='/robotsound_jp')
    data_list_node_strolling = rospy.get_param('~list_node_strolling', [])

    # read/write
    data_task_list = TaskList()
    data_task_executing = None

    class Ready(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['task_executing', 'strolling'])

        def execute(self, userdata):
            rospy.loginfo('Ready')

            if data_task_list.length() > 0 or data_task_executing is not None:
                return 'task_executing'

            return 'strolling'


    class Strolling(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['approaching', 'strolling'])

        def execute(self, userdata):
            rospy.loginfo('Strolling')

            # Move to a node randomly selected.
            next_target = random.choice(data_list_node_strolling)
            rospy.loginfo('Moving to {}'.format(next_target))
            result = data_spot_ros_client.execute_behaviors(
                next_target, blocking=True)

            # Searching a person.
            timeout = rospy.Time.now() + rospy.Duration(120)
            rate = rospy.Rate(5)
            while not rospy.is_shutdown() and rospy.Time.now() < timeout:
                rospy.loginfo('Searching person')
                rate.sleep()
                if get_nearest_person_pose() is not None:
                    return 'approaching'

            return 'strolling'


    class Approaching(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['ready','task_asking'])

        def execute(self, userdata):

            # approach
            timeout = rospy.Time.now() + rospy.Duration(30)
            while not rospy.is_shutdown():
                # check timeout
                if rospy.Time.now() > timeout:
                    rospy.logwarn('Timeout')
                    return 'ready'
                # get person pose
                pose = get_nearest_person_pose()
                if pose is None:
                    rospy.logwarn('No person')
                    continue
                elif calc_distance(pose.pose) < 1.0:
                    break
                else:
                    pos = PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                    theta = math.atan2(pos[1],pos[0])
                    pos = pos - 0.5 * pos / pos.Norm()
                    x = pos[0]
                    y = pos[1]
                    data_spot_ros_client.trajectory(x, y, theta, 2, blocking=True)

            return 'task_asking'


    class TaskAsking(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['ready'])
            self.actionclient_pickup_package = actionlib.SimpleActionClient(
                '~pickup_package', PickupPackageAction)
            self.actionclient_pickup_package.wait_for_server(rospy.Duration(5))

        def ask_task(self):

            self.actionclient_pickup_package.send_goal(
                PickupPackageGoal(timeout=rospy.Duration(30)))
            self.actionclient_pickup_package.wait_for_result()
            result = self.actionclient_pickup_package.get_result()
            return result.success, result.task.target_node_id, result.task.package_content, result.task.sender

        def execute(self, userdata):
            rospy.loginfo('TaskAsking')

            global data_task_list

            #

            # ask
            success, target_node_id, content, sender = self.ask_task()
            if success:
                data_task_list.append(Task(target_node_id, content, sender))

            return 'ready'


    class TaskExecuting(smach.State):

        def __init__(self):
            smach.State.__init__(
                self, outcomes=['ready'])
            self.actionclient_deliver_to = actionlib.SimpleActionClient(
                '~deliver_to', DeliverToAction)
            self.actionclient_deliver_to.wait_for_server(rospy.Duration(5))

        def execute(self, userdata):
            rospy.loginfo('TaskExecuting')

            global data_task_list
            global data_task_executing

            if data_task_executing is None and data_task_list.length() == 0:
                rospy.logwarn('No task left')
                return 'ready'
            elif data_task_executing is None and data_task_list.length() != 0:
                data_task_executing = data_task_list.pop()
            # else: execute data_task_executing

            success = self.do_deliver_to(
                                data_task_executing.target_node_id,
                                data_task_executing.content,
                                data_task_executing.sender
                                        )
            if success:
                data_task_executing = None

            return 'task_asking'

        def do_deliver_to(self, target_node_id, content, sender):

            goal = DeliverToGoal()
            goal.task.target_node_id = target_node_id
            goal.task.package_content = content
            goal.task.sender = sender
            self.actionclient_deliver_to.send_goal(goal)
            self.actionclient_deliver_to.wait_for_result()
            result = self.actionclient_deliver_to.get_result()
            return result.success

    with sm:
        smach.StateMachine.add('Ready', Ready(),
                               transitions={'task_executing':'TaskExecuting',
                                            'strolling':'Strolling'})
        smach.StateMachine.add('Strolling',Strolling(),
                               transitions={'approaching':'Approaching',
                                            'strolling':'Strolling'})
        smach.StateMachine.add('Approaching',Approaching(),
                               transitions={'ready':'Ready',
                                            'task_asking':'TaskAsking'})
        smach.StateMachine.add('TaskAsking',TaskAsking(),
                               transitions={'ready':'Ready'})
        smach.StateMachine.add('TaskExecuting',TaskExecuting(),
                               transitions={'ready':'Ready'})


    rospy.loginfo('initialized')

    outcome = sm.execute()

    rospy.loginfo('outcome: {}'.format(outcome))


if __name__ == '__main__':
    main()
