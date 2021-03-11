#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from franka_gripper.msg import GraspActionGoal
from sound_play.libsoundplay import SoundClient

BASE_LINK_NAME = "dual_arm_base"

class RwtCommandToArmPose():
  LR   = ["l", "r"]
  LEFTRIGHT   = {"l" : "left", "r" : "right"}
  sgns = { "l" : 1, "r" : -1 }

  def __init__(self):
    rospy.init_node('RwtCommandToArmPose')
    ### subs
    self.clk_pt_sub  = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, self.clk_pt_cb)
    self.cmd_str_sub = rospy.Subscriber("/rwt_command_string", String, self.cmd_str_cb)
    self.eew_sub     = {lr : rospy.Subscriber( "/dual_panda/"+lr+"arm_state_controller/F_ext", WrenchStamped, self.eew, lr) for lr in self.LR}
    self.eew         = {lr : WrenchStamped for lr in self.LR}
    self.tfl         = tf.TransformListener()
    ### pubs
    self.eep_pub     = {lr : rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1) for lr in self.LR}
    self.eep         = {lr :
                        PoseStamped(
                            header = Header(frame_id=BASE_LINK_NAME),
                            pose   = Pose(
                                position    = Point(0.5, self.sgns[lr] * 0.3, 1.0),
                                orientation = Quaternion_from_rpy(0, math.radians(90) ,math.radians(180)))) for lr in self.LR}
    self.g_pub       = {lr : rospy.Publisher('/dual_panda/'+lr+'arm/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1) for lr in self.LR}
    self.hp_pub      = rospy.Publisher('/master_head_pose', PoseStamped, queue_size=1)
    self.hp          = PoseStamped(header=Header(frame_id=BASE_LINK_NAME))
    self.fb_str_pub  = rospy.Publisher('/rwt_current_state', String, queue_size=1)
    self.lr_for_clk  = "r" ## start with right handed
    self.manip_mode  = {lr : "table" for lr in self.LR}
    ### Notification sound
    self.sound = SoundClient()

    rospy.loginfo("start main loop")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.fb_str_pub.publish(self.lr_for_clk+"_forclick")
      rate.sleep()

  def clk_pt_cb(self, msg):
    try:
      self.tfl.waitForTransform("/"+BASE_LINK_NAME, "/rwt_clicked_point", msg.header.stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/"+BASE_LINK_NAME, "/rwt_clicked_point", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return
    self.eep[self.lr_for_clk].header.stamp  = msg.header.stamp

    if self.manip_mode[self.lr_for_clk] == "table":
      self.eep[self.lr_for_clk].pose.position = Point( pos[0], pos[1], pos[2] + 0.1) ## hanko demo
    else:
      self.eep[self.lr_for_clk].pose.position = Point( pos[0]-0.05, pos[1], pos[2]) ## microwave demo
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

    if cmd == "look":
      q = self.hp.pose.orientation
      r, p, y = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
      delta = math.radians(30)
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

    lr = tgt

    if cmd == "clickteleopmode":
      self.lr_for_clk = lr

    if cmd == "gripper":
      gripper_cmd = GraspActionGoal()
      gripper_cmd.goal.width = ( 0.0 if val == "close" else 0.08)
      gripper_cmd.goal.epsilon.inner = 0.0
      gripper_cmd.goal.epsilon.outer = (0.08 if val == "close" else 0.0)
      gripper_cmd.goal.speed = 1 # [m/s]?
      gripper_cmd.goal.force = 10 # [N]
      self.g_pub[lr].publish(gripper_cmd)

    if cmd == "jogpos":
      delta = 0.01
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

    if cmd == "pull":
      self.eep[lr].pose.position.x -= 0.3
      self.eep[lr].pose.position.z -= 0.2
      self.eep[lr].header.stamp = rospy.Time.now()
      self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "push":
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
            self.eep[lr].pose.position.z -= 0.02 ## for hanko demo
          else:
            self.eep[lr].pose.position.x += 0.02
          self.eep[lr].header.stamp = rospy.Time.now()
          self.eep_pub[lr].publish(self.eep[lr])
          self.fb_str_pub.publish(lr+"_push_doing")
          rospy.sleep(0.5)
      else:
        rospy.logwarn("Fail @ " + str(f_norm) + " [N]")
        self.fb_str_pub.publish(lr+"_push_fail")
        self.sound.play(3)

      rospy.loginfo("Return")
      self.eep[lr].pose.position = pos_org
      self.eep[lr].header.stamp  = rospy.Time.now()
      self.eep_pub[lr].publish(self.eep[lr])

    # if cmd == "turn":
    #   q_org = self.eep[lr].pose.orientation
    #   q_rel = tf.transformations.quaternion_from_euler( math.radians( 30 if val == "r" else -30 ) ,0 ,0)
    #   # q_new = tf.transformations.quaternion_multiply(q_rel, [q_org.x, q_org.y, q_org.z, q_org.w])
    #   q_new = tf.transformations.quaternion_multiply([q_org.x, q_org.y, q_org.z, q_org.w], q_rel)
    #   self.eep[lr].pose.orientation = Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])
    #   self.eep[lr].header.stamp  = rospy.Time.now()
    #   self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "tablemode":
      if val == "1":
        self.manip_mode[lr]           = "table"
        self.eep[lr].pose.orientation = Quaternion_from_rpy(0, math.radians(90) ,math.radians(180))
        self.eep[lr].header.stamp     = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])
      if val == "2":
        self.manip_mode[lr]           = "table"
        self.eep[lr].pose.orientation = Quaternion_from_rpy(0, math.radians(90), math.radians(-90))
        self.eep[lr].header.stamp     = rospy.Time.now()
        self.eep_pub[lr].publish(self.eep[lr])

    if cmd == "frontmode":
      self.manip_mode[lr]           = "front"
      self.eep[lr].pose.orientation = Quaternion_from_rpy(0,0,0)
      self.eep[lr].header.stamp     = rospy.Time.now()
      self.eep_pub[lr].publish(self.eep[lr])


def Quaternion_from_rpy(r, p, y):
  q = tf.transformations.quaternion_from_euler( r, p ,y)
  return Quaternion(q[0], q[1], q[2], q[3])

if __name__ == '__main__':
  inst = RwtCommandToArmPose()
