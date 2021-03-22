#!/usr/bin/python
# -*- coding: utf-8 -*-
from dual_arm_rwt_command_parser import *
from franka_gripper.msg import GraspActionGoal

class DualPandaRwtCommandParser(DualArmRwtCommandParser):
  def __init__(self):
    self.node_name = "DualPandaRwtCommandParser"
    self.base_link_name = "dual_arm_base"
    self.dual_arm_distance = 0.2
    self.init_pose_xyz  = {lr : (0.5, self.sgns[lr] * 0.3, 1.0)          for lr in self.LR}
    self.init_pose_rpy  = {lr : (0, math.radians(90), math.radians(180)) for lr in self.LR}

    self.table_pose_rpy = {lr : (0, math.radians(90), math.radians(180)) for lr in self.LR}
    self.front_pose_rpy = self.table_pose_rpy ### front pose by panda is prohibit
    super(DualPandaRwtCommandParser, self).__init__()
    ### gripper override
    self.g_pub       = {lr : rospy.Publisher('/dual_panda/'+lr+'arm/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1) for lr in self.LR}
    self.loop()
    
  def set_gripper_close(self, _lr, _go_close):
    g_cmd = GraspActionGoal()
    g_cmd.goal.width = (0.0 if _go_close else 0.08)
    g_cmd.goal.epsilon.inner = 0.0
    g_cmd.goal.epsilon.outer = (0.08 if _go_close else 0.0)
    g_cmd.goal.speed = 1 # [m/s]?
    g_cmd.goal.force = 10 # [N]
    self.g_pub[_lr].publish(g_cmd)

if __name__ == '__main__':
  inst = DualPandaRwtCommandParser()
