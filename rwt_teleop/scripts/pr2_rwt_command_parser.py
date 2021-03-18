#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal
from sound_play.libsoundplay import SoundClient

class RwtCommandToArmPose():
  LR   = ["l", "r"]
  LEFTRIGHT   = {"l" : "left", "r" : "right"}
  sgns = { "l" : 1, "r" : -1 }

  def __init__(self):
    rospy.init_node('RwtCommandToArmPose')
    ### subs
    self.clk_pt_sub  = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, self.clk_pt_cb)
    self.cmd_str_sub = rospy.Subscriber("/rwt_command_string", String, self.cmd_str_cb)
    self.eew_sub     = {lr : rospy.Subscriber( "/"+self.LEFTRIGHT[lr]+"_endeffector/wrench", WrenchStamped, self.eew, lr) for lr in self.LR}
    self.eew         = {lr : WrenchStamped for lr in self.LR}
    self.tfl         = tf.TransformListener()
    ### pubs
    self.br = tf.TransformBroadcaster()
    self.eep_pub     = {lr : rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1) for lr in self.LR}
    self.eep         = {lr :
                        PoseStamped(
                            header = Header(frame_id="base_link"),
                            pose   = Pose(
                                position    = Point(0.5, self.sgns[lr] * 0.3, 1.0),
                                orientation = Quaternion(0,0,0,1))) for lr in self.LR}
    self.g_pub       = {lr : rospy.Publisher('/'+lr+'_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal, queue_size=1) for lr in self.LR}
    self.hp_pub      = rospy.Publisher('/master_head_pose', PoseStamped, queue_size=1)
    self.hp          = PoseStamped(header=Header(frame_id="base_link"))
    self.fb_str_pub  = rospy.Publisher('/rwt_current_state', String, queue_size=1)
    self.lr_for_clk  = "r" ## start with right handed
    self.is_dual = True
    self.manip_mode  = {lr : "front" for lr in self.LR}
    ### Notification sound
    self.sound = SoundClient()

    rospy.loginfo("start main loop")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      print self.is_dual
      self.fb_str_pub.publish(("d" if self.is_dual else self.lr_for_clk)+"_forclick")
      rate.sleep()

  def clk_pt_cb(self, msg):
    try:
      self.tfl.waitForTransform("/base_link", "/rwt_clicked_point", msg.header.stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/base_link", "/rwt_clicked_point", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return

    if self.is_dual:
      pos[0] = min(max( 0.4, pos[0]), 0.8)
      pos[1] = min(max(-0.3, pos[1]), 0.3)
      pos[2] = min(max( 0.8, pos[2]), 1.2)
      self.br.sendTransform((pos[0], pos[1], pos[2]), 
                            tf.transformations.quaternion_from_euler(0, 0, math.atan2(pos[1], pos[0])), 
                            msg.header.stamp, "/dual_arm_center", "/base_link")
      self.br.sendTransform((0,  0.08, 0), 
                            tf.transformations.quaternion_from_euler(math.radians(90), 0 ,0), 
                            msg.header.stamp, "/larm_in_dual_mode",  "/dual_arm_center")
      self.br.sendTransform((0, -0.08, 0), 
                            tf.transformations.quaternion_from_euler(math.radians(90), 0 ,0), 
                            msg.header.stamp, "/rarm_in_dual_mode",  "/dual_arm_center")
      for lr in self.LR:
        try:
          self.tfl.waitForTransform("/base_link", "/"+lr+"arm_in_dual_mode", msg.header.stamp, timeout=rospy.Duration(1))
          (pos, rot) = self.tfl.lookupTransform("/base_link", "/"+lr+"arm_in_dual_mode", rospy.Time(0))
        except Exception, e:
          rospy.logerr(e)
          return
        self.eep[lr].header.stamp  = msg.header.stamp
        self.eep[lr].pose.position = Point( pos[0], pos[1], pos[2])
        self.eep[lr].pose.orientation = Quaternion( rot[0], rot[1], rot[2], rot[3])
        self.eep_pub[lr].publish(self.eep[lr])

    else:
      if self.manip_mode == "table":
        self.eep[self.lr_for_cl].pose.position = Point( pos[0], pos[1], pos[2] + 0.1) ## hanko demo
      else:
        self.eep[self.lr_for_clk].pose.position = Point( pos[0]-0.05, pos[1], pos[2]) ## microwave demo
        self.eep[self.lr_for_clk].header.stamp  = msg.header.stamp
        self.eep_pub[self.lr_for_clk].publish(self.eep[self.lr_for_clk])

  def eew(self, msg, lr):
      self.eew[lr] = msg

  def cmd_str_cb(self, msg):
    rospy.loginfo("rwt_command_string [" + msg.data + "] received")
    str_l = msg.data.split("_")
    if len(str_l) == 2:
      tgt, cmd = str_l # l_open
    elif len(str_l) == 3:
      tgt, cmd, val = str_l # l_turn_l
    else:
      print "something wrong"
      return

    if tgt == "l" or tgt == "r":
      tgt_hand_list = [tgt]
    elif tgt == "d":
      tgt_hand_list = ["l", "r"]
    else:
      tgt_hand_list = []

    if tgt == "h" and cmd == "look":
      q = self.hp.pose.orientation
      r, p, y = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
      delta = math.radians(15)
      if val == "l": y += delta
      if val == "r": y -= delta
      if val == "u": p -= delta
      if val == "d": p += delta
      if val == "c": r, p, y = 0, 0, 0
      p = min(max(p, math.radians(-80)), math.radians(80))
      y = min(max(y, math.radians(-170)), math.radians(170))
      q = tf.transformations.quaternion_from_euler(r,p,y)
      self.hp.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
      self.hp.header.stamp     = rospy.Time.now()
      self.hp_pub.publish(self.hp)

    if tgt == "b" and cmd == "basepos":
      delta = 0.1
      if val == "+x": self.hp.pose.position.x += delta
      if val == "-x": self.hp.pose.position.x -= delta
      if val == "+y": self.hp.pose.position.y += delta
      if val == "-y": self.hp.pose.position.y -= delta
      if val == "+z": self.hp.pose.position.z += delta
      if val == "-z": self.hp.pose.position.z -= delta
      self.hp.pose.position.z = min(max(self.hp.pose.position.z, -0.2), 0.1)
      self.hp.header.stamp  = rospy.Time.now()
      self.hp_pub.publish(self.hp)

    if cmd == "clickteleopmode":
      if tgt in self.LR:
        self.lr_for_clk = tgt
        self.is_dual = False
      elif tgt == "d":
        self.is_dual = True

    if cmd == "gripper":
      gripper_cmd = Pr2GripperCommandActionGoal()
      gripper_cmd.goal.command.position   = ( 0.0 if val == "close" else 0.1)
      gripper_cmd.goal.command.max_effort = ( 25 if tgt == "d" else 75)
      for lr in tgt_hand_list:
        self.g_pub[lr].publish(gripper_cmd)

    if cmd == "jogpos":
      delta = 0.01
      for lr in tgt_hand_list:
        if val == "+x": self.eep[lr].pose.position.x += delta
        if val == "-x": self.eep[lr].pose.position.x -= delta
        if val == "+y": self.eep[lr].pose.position.y += delta
        if val == "-y": self.eep[lr].pose.position.y -= delta
        if val == "+z": self.eep[lr].pose.position.z += delta
        if val == "-z": self.eep[lr].pose.position.z -= delta
        self.eep[lr].header.stamp  = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "jogrot":
      delta = math.radians(10)
      for lr in tgt_hand_list:
        if val == "+x": q_rel = tf.transformations.quaternion_from_euler( delta, 0, 0)
        if val == "-x": q_rel = tf.transformations.quaternion_from_euler(-delta, 0, 0)
        if val == "+y": q_rel = tf.transformations.quaternion_from_euler(0,  delta, 0)
        if val == "-y": q_rel = tf.transformations.quaternion_from_euler(0, -delta, 0)
        if val == "+z": q_rel = tf.transformations.quaternion_from_euler(0, 0,  delta)
        if val == "-z": q_rel = tf.transformations.quaternion_from_euler(0, 0, -delta)
        q_org = self.eep[lr].pose.orientation
        q_new = tf.transformations.quaternion_multiply(q_rel, [q_org.x, q_org.y, q_org.z, q_org.w])
        self.eep[lr].pose.orientation = Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])
        self.eep[lr].header.stamp  = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "push":
      for lr in tgt_hand_list:
        pos_org = copy.deepcopy(self.eep[lr].pose.position)
        f_norm = 0
        for i in range(10):
          f_norm = np.linalg.norm([self.eew[lr].wrench.force.x, self.eew[lr].wrench.force.y, self.eew[lr].wrench.force.z])
          if f_norm > float(val):
            rospy.loginfo("Hit @ " + str(f_norm) + " [N]")
            self.fb_str_pub.publish(lr+"_push_ok")
            self.sound.play(1)
            break
          else:
            rospy.loginfo("Go @ " + str(f_norm) + " [N]")
            if self.manip_mode[lr] == "table":
              self.eep[lr].pose.position.z -= 0.005 ## for hanko demo
            else:
              self.eep[lr].pose.position.x += 0.01
              self.eep[lr].header.stamp = rospy.Time.now()
              self.eep_pub[lr].publish(self.eep[lr])
              self.fb_str_pub.publish(lr+"_push_doing")
              rospy.sleep(1)
        else:
          rospy.logwarn("Fail @ " + str(f_norm) + " [N]")
          self.fb_str_pub.publish(lr+"_push_fail")
          self.sound.play(3)
        rospy.loginfo("Return")
        self.eep[lr].pose.position = pos_org
        self.eep[lr].header.stamp  = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "tablemode":
      for lr in tgt_hand_list:
        self.manip_mode[lr]           = "table"
        self.eep[lr].pose.orientation = Quaternion_from_rpy(0, math.radians(90) ,0)
        self.eep[lr].header.stamp     = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "frontmode":
      for lr in tgt_hand_list:
        self.manip_mode[lr]           = "front"
        self.eep[lr].pose.orientation = Quaternion_from_rpy(0,0,0)
        self.eep[lr].header.stamp     = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])


def Quaternion_from_rpy(r, p, y):
  q = tf.transformations.quaternion_from_euler( r, p ,y)
  return Quaternion(q[0], q[1], q[2], q[3])

if __name__ == '__main__':
  inst = RwtCommandToArmPose()
