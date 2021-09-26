#!/usr/bin/env python
# -*- encoding: utf-8 -*-

"""this script is a demo that spot go to a specified spot"""

import actionlib
import rospy
from spot_behavior_manager_msgs.msg import LeadPersonAction, LeadPersonGoal


def main():

    rospy.init_node('spot_go_to_spot')

    target_node_id = rospy.get_param('~target_node_id', 'eng2_7FElevator')
    num_retry = rospy.get_param('~num_retry', 3)

    client = actionlib.SimpleActionClient(
        '/spot_behavior_manager_demo/execute_behaviors', LeadPersonAction)

    rospy.loginfo('waiting for server...')

    if not client.wait_for_server(rospy.Duration(10)):
        rospy.logerr('Server down.')
        return

    rospy.loginfo('Start to go to {}'.format(target_node_id))

    for i in range(num_retry):

        client.send_goal_and_wait(
            LeadPersonGoal(target_node_id=target_node_id))
        result = client.get_result()
        if result.success:
            rospy.loginfo('Finished.')
            return
        else:
            rospy.logwarn('Failed. retrying...')

    rospy.logerr('Failed even after retrying.')


if __name__ == '__main__':
    main()
