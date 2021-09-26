#!/usr/bin/env python
# -*- encoding: utf-8 -*-

""""""

import sys
import actionlib
from sound_play.libsoundplay import SoundClient
import rospy
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal


class Demo:

    def __init__(self):

        self.node_id_73b2 = rospy.get_param('~node_id_73b2', 'eng2_73B2')
        self.node_id_mech_office = rospy.get_param(
            '~node_id_mech_office', 'eng2_Mech_Office')
        self.num_max_retry = rospy.get_param('~num_max_retry', 3)

        self.actionclient = actionlib.SimpleActionClient(
            '/spot_behavior_manager_demo/execute_behaviors', LeadPersonAction)
        self.soundclient = SoundClient(sound_action='/robotsound_jp',sound_topic='/robotsound_jp')

        rospy.loginfo('waiting for server...')
        if not self.actionclient.wait_for_server(rospy.Duration(10)):
            rospy.logerr('Server down.')
            self.initialized = False
        else:
            self.initialized = True

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
                    sys.exit(1)

        rospy.loginfo('I am back to 73B2.')
        sys.exit(1)

    def run(self):

        # wait for envelop
        self.soundclient.say('事務室へ持っていくものを置いてください。',blocking=True)
        ## checking if something is placed on the robot
        ### TODO
        ## ask if OK
        ### TODO

        # go to mech office
        rospy.loginfo('Start to go to {}'.format(self.node_id_mech_office))
        self.soundclient.say('事務室へ向かいます',blocking=True)
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

        self.soundclient.say('荷物を回収してください。',blocking=True)
        ## checking if package is taken from the robot
        ### TODO
        ## ask if OK
        ### TODO

        # go back to 73b2
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


def main():
    rospy.init_node('spot_go_to_spot')
    demo = Demo()
    if not demo.initialized:
        sys.exit(1)
    demo.run()


if __name__ == '__main__':
    main()
