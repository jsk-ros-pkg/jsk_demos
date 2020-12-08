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
  LR   = ["l", "r"]
  sgns = { "l" : 1, "r" : -1 }

  def __init__(self):
    rospy.init_node('RwtCommandToArmPose')
    self.ee_pose      = {}
    self.ep_pub       = {}
    self.g_pub        = {}
    self.tfl = tf.TransformListener()
    self.cmd_str_sub = rospy.Subscriber("/rwt_command_string", String, self.cmd_str_cb)
    self.sub = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, self.click_cb)

    for lr in self.LR:
      self.ee_pose[lr] = PoseStamped(header=Header(frame_id="base_link"))
      self.ee_pose[lr].pose.position    = Point(0.5, self.sgns[lr] * 0.3, 1.0)
      self.ee_pose[lr].pose.orientation = Quaternion(0,0,0,1)
      self.ep_pub[lr]  = rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1)
      self.g_pub[lr]   = rospy.Publisher('/'+lr+'_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal, queue_size=1)
  
    self.head_pose     = PoseStamped(header=Header(frame_id="base_link"))
    self.head_pub      = rospy.Publisher('/master_head_pose', PoseStamped, queue_size=1)
    self.lr_for_click  = "l"
    self.lr_mode_pub   = rospy.Publisher('/rwt_current_state', String, queue_size=1)

    rospy.loginfo("start main loop")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.lr_mode_pub.publish(self.lr_for_click)
      rate.sleep()


  def click_cb(self, msg):
    try:
      self.tfl.waitForTransform("/base_link", "/rwt_clicked_point", msg.header.stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/base_link", "/rwt_clicked_point", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return

    self.ee_pose[self.lr_for_click].header.stamp  = msg.header.stamp
    self.ee_pose[self.lr_for_click].pose.position = Point( pos[0], pos[1], pos[2])
    self.ep_pub[self.lr_for_click].publish(self.ee_pose[self.lr_for_click])


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
      q = self.head_pose.pose.orientation
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
      self.head_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
      self.head_pose.header.stamp     = rospy.Time.now()
      self.head_pub.publish(self.head_pose)

    lr = tgt

    if cmd == "clickteleopmode":
      self.lr_for_click = lr

    if cmd == "gripper":
      gripper_cmd = Pr2GripperCommandActionGoal()
      gripper_cmd.goal.command.position = ( 0.0 if val == "close" else 0.1)
      gripper_cmd.goal.command.max_effort = 75
      self.g_pub[lr].publish(gripper_cmd)

    if cmd == "jogpos":
      delta = 0.01
      if val == "+x": self.ee_pose[lr].pose.position.x += delta
      if val == "-x": self.ee_pose[lr].pose.position.x -= delta
      if val == "+y": self.ee_pose[lr].pose.position.y += delta
      if val == "-y": self.ee_pose[lr].pose.position.y -= delta
      if val == "+z": self.ee_pose[lr].pose.position.z += delta
      if val == "-z": self.ee_pose[lr].pose.position.z -= delta
      self.ee_pose[lr].header.stamp  = rospy.Time.now()
      self.ep_pub[lr].publish(self.ee_pose[lr])

    if cmd == "jogrot":
      delta = math.radians(10)
      if val == "+x": q_rel = tf.transformations.quaternion_from_euler( delta, 0, 0)
      if val == "-x": q_rel = tf.transformations.quaternion_from_euler(-delta, 0, 0)
      if val == "+y": q_rel = tf.transformations.quaternion_from_euler(0,  delta, 0)
      if val == "-y": q_rel = tf.transformations.quaternion_from_euler(0, -delta, 0)
      if val == "+z": q_rel = tf.transformations.quaternion_from_euler(0, 0,  delta)
      if val == "-z": q_rel = tf.transformations.quaternion_from_euler(0, 0, -delta)
      q_org = self.ee_pose[lr].pose.orientation
      q_new = tf.transformations.quaternion_multiply(q_rel, [q_org.x, q_org.y, q_org.z, q_org.w])
      self.ee_pose[lr].pose.orientation = Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])
      self.ee_pose[lr].header.stamp  = rospy.Time.now()
      self.ep_pub[lr].publish(self.ee_pose[lr])

    if cmd == "pull":
      self.ee_pose[lr].pose.position.x -= 0.3
      self.ee_pose[lr].pose.position.z -= 0.2
      self.ee_pose[lr].header.stamp  = rospy.Time.now()
      self.ep_pub[lr].publish(self.ee_pose[lr])

    if cmd == "turn":
      q_org = self.ee_pose[lr].pose.orientation
      q_rel = tf.transformations.quaternion_from_euler( math.radians( 30 if val == "r" else -30 ) ,0 ,0)
      # q_new = tf.transformations.quaternion_multiply(q_rel, [q_org.x, q_org.y, q_org.z, q_org.w])
      q_new = tf.transformations.quaternion_multiply([q_org.x, q_org.y, q_org.z, q_org.w], q_rel)
      self.ee_pose[lr].pose.orientation = Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])
      self.ee_pose[lr].header.stamp  = rospy.Time.now()
      self.ep_pub[lr].publish(self.ee_pose[lr])

    if cmd == "tabletopmode":
      q_tt = tf.transformations.quaternion_from_euler( 0, math.radians(90) ,0)
      self.ee_pose[lr].pose.orientation = Quaternion(q_tt[0],q_tt[1],q_tt[2],q_tt[3])
      self.ee_pose[lr].header.stamp  = rospy.Time.now()
      self.ep_pub[lr].publish(self.ee_pose[lr])


if __name__ == '__main__':
  inst = RwtCommandToArmPose()
