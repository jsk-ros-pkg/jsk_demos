#!/usr/bin/python
# -*- coding: utf-8 -*-

import math

import rospy

from rwt_teleop.dual_arm_rwt_command_parser import DualArmRwtCommandParser

from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal


class PR2RwtCommandParser(DualArmRwtCommandParser):
    def __init__(self):
        self.node_name = "PR2RwtCommandParser"
        self.base_link_name = "base_link"
        self.dual_arm_distance = 0.16
        self.init_pose_xyz = {}
        self.init_pose_rpy = {}
        self.front_pose_rpy = {}
        self.table_pose_rpy = {}
        for lr in self.LR:
            self.init_pose_xyz[lr] = (0.5, self.sgns[lr] * 0.3, 1.0)
            self.init_pose_rpy[lr] = (0, 0, 0)
            self.front_pose_rpy[lr] = (0, 0, 0)
            self.table_pose_rpy[lr] = (0, math.radians(90), 0)
        super(PR2RwtCommandParser, self).__init__()
        # gripper override
        self.g_pub = {}
        for lr in self.LR:
            self.g_pub[lr] = rospy.Publisher(
                '/'+lr+'_gripper_controller/gripper_action/goal',
                Pr2GripperCommandActionGoal, queue_size=1)
        self.loop()

    def set_gripper_close(self, _lr, _go_close):
        g_cmd = Pr2GripperCommandActionGoal()
        g_cmd.goal.command.position = (0.0 if _go_close else 0.1)
        g_cmd.goal.command.max_effort = 75
        self.g_pub[_lr].publish(g_cmd)


if __name__ == '__main__':
    inst = PR2RwtCommandParser()
