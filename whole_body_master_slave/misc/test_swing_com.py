#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
import signal
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  pub_com = rospy.Publisher('/master_com_pose', PoseStamped, queue_size=1)
  pub_rf  = rospy.Publisher('/master_rleg_pose',  PoseStamped, queue_size=1)
  pub_lf  = rospy.Publisher('/master_lleg_pose',  PoseStamped, queue_size=1)
  pub_hz  = rospy.Publisher('/test_swing_com_hz',  Float32, queue_size=1)
  
  rospy.init_node('test_swing_com', anonymous=True)
  HZ = 100.0
  r = rospy.Rate(HZ)
  
  pub_val_com = PoseStamped()
  pub_val_rf  = PoseStamped()
  pub_val_lf  = PoseStamped()
  pub_val_hz  = Float32()

  loop = 0
  while not rospy.is_shutdown():
    rospy.loginfo_throttle(1, "pub init")
    pub_val_com.header.stamp = rospy.Time.now()
    pub_val_rf.header.stamp = rospy.Time.now()
    pub_val_lf.header.stamp = rospy.Time.now()
    pub_com.publish(pub_val_com)
    pub_rf.publish(pub_val_rf)
    pub_lf.publish(pub_val_lf)
    pub_hz.publish(pub_val_hz)
    r.sleep()
    loop += 1
    if loop > 3 * HZ:
      break
  
  loop = 0
  phase = 0.0;
  rospy.loginfo("start pub coil COM trajectory=")
  while not rospy.is_shutdown():
    sec = loop/HZ
    rad = 0.1
    com_hz = 0.01*sec
    phase += 2*math.pi*com_hz*(1/HZ)
    rospy.loginfo_throttle(1, "com_hz="+ str(com_hz))
    # pub_val_com.pose.position.y = rad * math.sin(phase)
    pub_val_com.pose.position.y = rad * ( 1.0 if math.sin(phase)>0 else -1.0)
    pub_val_rf.pose.position.z  = rad * ( 1.0 if math.sin(phase)>0 else 0.0)
    pub_val_lf.pose.position.z  = rad * ( 1.0 if math.sin(phase)<0 else 0.0)
    pub_val_hz.data = com_hz

    pub_val_com.header.stamp = rospy.Time.now()
    pub_val_rf.header.stamp = rospy.Time.now()
    pub_val_lf.header.stamp = rospy.Time.now()
    pub_com.publish(pub_val_com)
    pub_rf.publish(pub_val_rf)
    pub_lf.publish(pub_val_lf)
    pub_hz.publish(pub_val_hz)

    r.sleep()
    loop += 1
