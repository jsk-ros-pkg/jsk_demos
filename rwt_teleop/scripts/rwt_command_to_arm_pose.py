#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal

class RwtCommandToArmPose():
  LR          = ["l", "r"]
  sgns        = { "l" : 1, "r" : -1 }

  def __init__(self):
    rospy.init_node('RwtCommandToArmPose')
    self.ee_pose         = {}
    self.ep_pub          = {}
    self.g_pub           = {}
    self.current_lr_mode = "l"
    self.pan = 0
    self.tilt = 0

    self.tfl = tf.TransformListener()
    self.cmd_str_sub = rospy.Subscriber("/rwt_command_string", String, self.cmd_str_cb)
    self.sub = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, self.click_cb)

    for lr in self.LR:
      self.ee_pose[lr] = PoseStamped()
      self.ee_pose[lr].header.frame_id  = "base_link"
      self.ee_pose[lr].pose.position    = Point(0.5, self.sgns[lr] * 0.3, 1.0)
      self.ee_pose[lr].pose.orientation = Quaternion(0,0,0,1)
      self.ep_pub[lr]  = rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1)
      self.g_pub[lr]   = rospy.Publisher('/'+lr+'_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal, queue_size=1)
    
    self.head_pub      = rospy.Publisher('/master_head_pose', PoseStamped, queue_size=1)
    self.head_pose     = PoseStamped()

    rospy.loginfo("start main loop")
    rospy.spin()

  def click_cb(self, msg):
    try:
      self.tfl.waitForTransform("/base_link", "/rwt_clicked_point", msg.header.stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/base_link", "/rwt_clicked_point", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return

    self.ee_pose[self.current_lr_mode].header.frame_id = "base_link"
    self.ee_pose[self.current_lr_mode].header.stamp    = msg.header.stamp
    self.ee_pose[self.current_lr_mode].pose.position   = Point( pos[0], pos[1], pos[2])
    # self.clk_tgt_pos[self.current_lr_mode].pose.orientation = Quaternion(0,0,0,1)
    self.ep_pub[self.current_lr_mode].publish(self.ee_pose[self.current_lr_mode])


  def cmd_str_cb(self, msg):
    rospy.loginfo("rwt_command_string [" + msg.data + "] received")
    str_l = msg.data.split("_")
    if len(str_l) == 2:
      tgt, cmd = str_l # l_open
    elif len(str_l) == 3:
      tgt, cmd, val = str_l # l_turn_90
    else:
      print "something wrong"
      return

    if cmd == "look":
      q = self.head_pose.pose.orientation
      r, p, y = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
      
      if tgt == "l": y += math.pi / 6
      if tgt == "r": y -= math.pi / 6
      if tgt == "u": p -= math.pi / 6
      if tgt == "d": p += math.pi / 6
      if tgt == "c": r, p, y = 0, 0, 0
      
      p = min(max(p, math.radians(-80)), math.radians(80))
      y = min(max(y, math.radians(-170)), math.radians(170))
      q = tf.transformations.quaternion_from_euler(r,p,y)
      self.head_pose.header.frame_id  = "base_link"
      self.head_pose.header.stamp     = rospy.Time.now()
      self.head_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
      self.head_pub.publish(self.head_pose)

    lr = tgt
      
    if cmd == "mode":
      self.current_lr_mode = lr

    if cmd in ["open", "close"]:
      gripper_cmd = Pr2GripperCommandActionGoal()
      gripper_cmd.goal.command.position = ( 0.0 if cmd == "close" else 0.1)
      gripper_cmd.goal.command.max_effort = 75
      self.g_pub[lr].publish(gripper_cmd)

    if cmd == "pull":
      tmp = copy.deepcopy(self.ee_pose[lr])
      tmp.pose.position.x -= 0.3
      tmp.pose.position.z -= 0.2
      self.ep_pub[lr].publish(tmp)

    if cmd == "turn":
      q_org = self.ee_pose[lr].pose.orientation
      q_rel = tf.transformations.quaternion_from_euler(math.pi/2,0,0)
      q_new = tf.transformations.quaternion_multiply(q_rel, [q_org.x, q_org.y, q_org.z, q_org.w])
      self.ee_pose[lr].pose.orientation = Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])

if __name__ == '__main__':
  inst = RwtCommandToArmPose()
