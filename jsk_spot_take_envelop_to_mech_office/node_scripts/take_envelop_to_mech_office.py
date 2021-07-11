#!/usr/bin/env python
# -*- encoding: utf-8 -*-

""""""

import actionlib
import rospy
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal
from std_msgs.msg import String
import sys


def sendResultMail():

    last_node_id = rospy.wait_for_message('')

    return True


def sendRescueMail():

    return True


def main():

    rospy.init_node('spot_go_to_spot')
    demo = Demo()
    demo.run()


class Demo:

    def __init__(self):

        self.node_id_73b2 = rospy.get_param('~node_id_73b2', 'eng2_73B2')
        self.node_id_mech_office = rospy.get_param(
            '~node_id_mech_office', 'eng2_Mech_Office')
        self.num_max_retry = rospy.get_param('~num_max_retry', 3)
        self.mailaddress = rospy.get_param(
            '~mailaddress', 'spot@jsk.imi.i.u-tokyo.ac.jp')

        self.actionclient = actionlib.SimpleActionClient(
            '/spot_behavior_manager_demo/lead_person', LeadPersonAction)

        rospy.loginfo('waiting for server...')
        if not self.actionclient.wait_for_server(rospy.Duration(10)):
            rospy.logerr('Server down.')
            sys.exit(1)

    def exit(self):

        rospy.logwarn('Trying to go back to 73b2...')

        for num_trial in range(self.num_max_retry):
            self.actionclient.send_goal_and_wait(
                LeadPersonGoal(target_node_id=self.node_id_73b2))
            result = self.actionclient.get_result()
            if result.success:
                rospy.loginfo('Finished.')
                break
            else:
                if num_trial < self.num_max_retry - 1:
                    rospy.logwarn('Failed. retrying...')
                else:
                    rospy.logerr('Failed even after retrying.')
                    sendRescueMail()
                    self.exit(1)

        rospy.loginfo('I am back to 73B2.')
        sendResultMail()
        sys.exit(1)

    def run(self):

        rospy.loginfo('Start to go to {}'.format(self.node_id_mech_office))

        for num_trial in range(self.num_max_retry):
            self.actionclient.send_goal_and_wait(
                LeadPersonGoal(target_node_id=self.node_id_mech_office))
            result = self.actionclient.get_result()
            if result.success:
                rospy.loginfo('Finished.')
                break
            else:
                if num_trial < self.num_max_retry - 1:
                    rospy.logwarn('Failed. retrying...')
                else:
                    rospy.logerr('Failed even after retrying.')
                    self.exit()

        rospy.loginfo('I am now at the Mech Office.')

        rospy.loginfo('Return to {}'.format(self.node_id_73b2))

        for num_trial in range(self.num_max_retry):
            self.actionclient.send_goal_and_wait(
                LeadPersonGoal(target_node_id=self.node_id_73b2))
            result = self.actionclient.get_result()
            if result.success:
                rospy.loginfo('Finished.')
                break
            else:
                if num_trial < self.num_max_retry - 1:
                    rospy.logwarn('Failed. retrying...')
                else:
                    rospy.logerr('Failed even after retrying.')
                    self.exit()

        rospy.loginfo('I am back to 73B2.')
        sendResultMail()


if __name__ == '__main__':
    main()
