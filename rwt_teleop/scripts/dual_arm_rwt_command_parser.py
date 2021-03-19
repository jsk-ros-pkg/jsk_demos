#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
import math
import numpy as np
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import *
from sound_play.libsoundplay import SoundClient

class DualArmRwtCommandParser(object):
  LR   = ["l", "r"]
  LEFTRIGHT   = {"l" : "left", "r" : "right"}
  sgns = { "l" : 1, "r" : -1 }
  node_name = "RwtCommandParser"
  base_link_name = "base_link"
  dual_arm_distance = 0.16
  init_pose_xyz  = {"l" : (0.5, 0.3, 1.0), "r" : (0.5, -0.3, 1.0)}
  init_pose_rpy  = {lr : (0, 0, 0)                       for lr in LR}
  front_pose_rpy = {lr : (0, 0, 0)                       for lr in LR}
  table_pose_rpy = {lr : (0, math.radians(90), 0)        for lr in LR}

  def __init__(self):
    rospy.init_node(self.node_name)
    ### subs
    self.clk_pt_sub  = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, self.clk_pt_cb)
    self.cmd_str_sub = rospy.Subscriber("/rwt_command_string", String, self.cmd_str_cb)
    self.eew_sub     = {lr : rospy.Subscriber( lr+"_wrench", WrenchStamped, self.eew, lr) for lr in self.LR}
    self.eew         = {lr : WrenchStamped for lr in self.LR}
    self.tfl         = tf.TransformListener()
    ### pubs
    self.br = tf.TransformBroadcaster()
    self.eep_pub     = {lr : rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1) for lr in self.LR}
    self.eep         = {lr :
                        PoseStamped(
                          header = Header(frame_id = self.base_link_name),
                          pose   = Pose(
                            position    = Point(*self.init_pose_xyz[lr]),
                            orientation = Quaternion_from_rpy(*self.init_pose_rpy[lr]))) for lr in self.LR}
    self.g_pub       = {lr : None for lr in self.LR} # should be overrided
    self.hp_pub      = rospy.Publisher('/master_head_pose', PoseStamped, queue_size=1)
    self.hp          = PoseStamped(header=Header(frame_id = self.base_link_name))
    self.fb_str_pub  = rospy.Publisher('/rwt_current_state', String, queue_size=1)
    self.lrd_for_clk = "r" ## l=left, r=right, d=dual
    self.task_context = {lr : "table" for lr in self.LR}
    self.sound = SoundClient()

  def loop(self):
    rospy.loginfo("start main loop")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.fb_str_pub.publish((self.lrd_for_clk)+"_forclick")
      rate.sleep()

  def clk_pt_cb(self, msg):
    try:
      self.tfl.waitForTransform("/"+self.base_link_name, "/rwt_clicked_point", msg.header.stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/"+self.base_link_name, "/rwt_clicked_point", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return

    if self.lrd_for_clk == "d":
      pos[0] = min(max( 0.4, pos[0]), 0.8)
      pos[1] = min(max(-0.3, pos[1]), 0.3)
      pos[2] = min(max( 0.8, pos[2]), 1.2)
      self.br.sendTransform(pos, tf.transformations.quaternion_from_euler(0, 0, 0),
                            msg.header.stamp, "/dual_arm_center", "/"+self.base_link_name)

      for lr in lrd2lr_list(self.lrd_for_clk):
        self.br.sendTransform((0, self.sgns[lr] * self.dual_arm_distance/2.0, 0),
                              (self.eep[lr].pose.orientation.x,
                               self.eep[lr].pose.orientation.y,
                               self.eep[lr].pose.orientation.z,
                               self.eep[lr].pose.orientation.w), 
                              msg.header.stamp, "/"+lr+"arm_in_dual_mode",  "/dual_arm_center")
      
      for lr in self.LR:
        try:
          self.tfl.waitForTransform("/"+self.base_link_name, "/"+lr+"arm_in_dual_mode", msg.header.stamp, timeout=rospy.Duration(1))
          (pos, rot) = self.tfl.lookupTransform("/"+self.base_link_name, "/"+lr+"arm_in_dual_mode", rospy.Time(0))
        except Exception, e:
          rospy.logerr(e)
          return
        self.eep[lr].pose.position = Point( pos[0], pos[1], pos[2])
        self.eep[lr].pose.orientation = Quaternion( rot[0], rot[1], rot[2], rot[3])
        self.eep[lr].header.stamp  = msg.header.stamp
        self.eep_pub[lr].publish(self.eep[lr])

    else:
      for lr in lrd2lr_list(self.lrd_for_clk):
        if self.task_context[lr] == "table":
          self.eep[lr].pose.position = Point( pos[0], pos[1], pos[2] + 0.1) ## hanko demo
        else:
          self.eep[lr].pose.position = Point( pos[0] -0.1, pos[1], pos[2]) ## microwave demo
        self.eep[lr].header.stamp  = msg.header.stamp
        self.eep_pub[lr].publish(self.eep[lr])

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

    if cmd == "resetpose":
      for lr in self.LR:
        self.eep[lr].pose.orientation = Quaternion_from_rpy(*self.init_pose_rpy[lr])
      rospy.wait_for_service("set_reset_pose")
      try:
        srv = rospy.ServiceProxy("set_reset_pose", Empty)
        srv()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
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
      self.lrd_for_clk = tgt

    if cmd == "gripper":
      for lr in lrd2lr_list(tgt):
        self.set_gripper_close(lr, (val == "close"))

    if cmd == "jogpos":
      delta = 0.01
      for lr in lrd2lr_list(tgt):
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
      for lr in lrd2lr_list(tgt):
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
      for lr in lrd2lr_list(tgt):
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
            if self.task_context[lr] == "table":
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
      for lr in lrd2lr_list(tgt):
        self.task_context[lr] = "table"
        self.set_table_pose(lr)
        
    if cmd == "frontmode":
      for lr in lrd2lr_list(tgt):
        self.task_context[lr] = "front"
        self.set_front_pose(lr)

  def set_table_pose(self, _lr):
    self.eep[_lr].pose.orientation = Quaternion_from_rpy(*self.table_pose_rpy[_lr])
    self.eep[_lr].header.stamp     = rospy.Time.now()
    self.eep_pub[_lr].publish(self.eep[_lr])
        
  def set_front_pose(self, _lr):
    self.eep[_lr].pose.orientation = Quaternion_from_rpy(*self.front_pose_rpy[_lr])
    self.eep[_lr].header.stamp     = rospy.Time.now()
    self.eep_pub[_lr].publish(self.eep[_lr])

  def set_gripper_close(self, _lr, _go_close):
    g_cmd = Pr2GripperCommandActionGoal()
    g_cmd.goal.command.position   = (0.0 if _go_close else 0.1)
    g_cmd.goal.command.max_effort = 75
    self.g_pub[_lr].publish(g_cmd)

def lrd2lr_list(_lrd):
  if _lrd in ["l", "r"]:
    return [_lrd]
  elif _lrd == "d":
    return ["l", "r"]
  else:
    rospy.logerr
    return []

def Quaternion_from_rpy(r, p, y):
  q = tf.transformations.quaternion_from_euler( r, p ,y)
  return Quaternion(q[0], q[1], q[2], q[3])

if __name__ == '__main__':
  inst = DualArmCommandParser()
