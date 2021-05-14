#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerRequest
import sys

class Node(object):

    def __init__(self):

        self._duration_timeout = rospy.get_param('~duration_timeout', 10.0)
        self._follow_button = rospy.get_param('~follow_button', -1)
        self._duration_impersity = rospy.get_param('~duration_impersity', 1.0)

        self._is_pressed = False
        self._last_pressed = rospy.Time.now()

        #
        try:
            rospy.wait_for_service('~follow', timeout=rospy.Duration(self._duration_timeout))
        except Exception as e:
            rospy.logwarn('{}'.format(e))
            sys.exit(1)
        self._service_proxy = rospy.ServiceProxy('~follow',Trigger)

        #
        self._sub_joy = rospy.Subscriber('~joy', Joy, self._callback)

        rospy.loginfo('now inialized')

    def _callback(self,msg):

        if self._follow_button < len(msg.buttons):
            if msg.buttons[self._follow_button] > 0 and not self._is_pressed:
                # become pressed
                self._is_pressed = True
                rospy.loginfo('button pressed')
                if (rospy.Time.now() - self._last_pressed).to_sec() > self._duration_impersity:
                    self._last_pressed = rospy.Time.now()
                    self.callService()
            elif msg.buttons[self._follow_button] <= 0 and self._is_pressed:
                # become not pressed
                self._is_pressed = False
                rospy.loginfo('button releaseed')
        else:
            rospy.logwarn('button id is out of range')

    def callService(self):
        self._service_proxy()

if __name__ == '__main__':
    rospy.init_node('follow_on_button')
    node = Node()
    rospy.spin()
