#!/usr/bin/env python
# -*- encoding: utf-8 -*-

"""this script is a demo that spot go to a specified spot"""

import actionlib
import rospy

from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal


def main():

    rospy.init_node('spot_go_to_spot')

    target_node_id = rospy.get_param('~target_node_id', 'eng2_7FElevator')

    client = actionlib.SimpleActionClient(
        '/spot_behavior_manager_demo/lead_person', LeadPersonAction)

    rospy.loginfo('waiting for server...')

    if not client.wait_for_server(rospy.Duration(10)):
        rospy.logerr('Server down.')
        return

    rospy.loginfo('send a goal')

    client.send_goal_and_wait(LeadPersonGoal(target_node_id=target_node_id))

    rospy.loginfo('finished.')


if __name__ == '__main__':
    main()
