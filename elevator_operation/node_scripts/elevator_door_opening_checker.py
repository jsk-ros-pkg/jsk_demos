#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64
from elevator_operation.msg import DoorState


class ElevatorDoorOpeningCheckerNode(object):

    def __init__(self):

        self.threshold = rospy.get_param('~threshold', 100)
        self.pub = rospy.Publisher('~output', Int64, queue_size=1)
        self.pub_door_open = rospy.Publisher('~door_state', DoorState, queue_size=1)
        self.sub = rospy.Subscriber('~input', PointCloud2, self.callback)

    def callback(self, msg):

        rospy.logdebug('door points: {}'.format(len(msg.data)))
        self.pub.publish(Int64(data=len(msg.data)))
        if len(msg.data) < self.threshold:
            self.pub_door_open.publish(DoorState(state=DoorState.OPEN))
        else:
            self.pub_door_open.publish(DoorState(state=DoorState.CLOSE))


if __name__ == '__main__':

    rospy.init_node('elevator_door_opening_checker_node')
    node = ElevatorDoorOpeningCheckerNode()
    rospy.spin()
