#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import signal
import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Joy

pub_val_com = PointStamped()
pub_val_rf = PointStamped()
pub_val_lf = PointStamped()
pub_val_rh = PointStamped()
pub_val_lh = PointStamped()
pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh]

key_list = ["com", "rf", "lf", "rh", "lh"]
topic_d = {"com":PointStamped, "rf":PointStamped, "lf":PointStamped, "rh":PointStamped, "lh":PointStamped}

HZ = 100.0

rf_vel = [0.0, 0.0]
lf_vel = [0.0, 0.0]
commode = "IDLE"
rfon = lfon = Bool()
f_vel = -0.4/20
TH_XY = 0
TH_Z = 20
  
def callback_R(data):
  global rf_vel, rfon, f_vel
  if np.absolute(data.wrench.force.x) > TH_XY:
    rf_vel[0] = data.wrench.force.x * f_vel
  else:
    rf_vel[0] =0
  if np.absolute(data.wrench.force.y) > TH_XY:
    rf_vel[1] = data.wrench.force.y * f_vel
  else:
    rf_vel[1] =0
  if data.wrench.force.z > TH_Z:
    rfon = True
  else:
    rfon = False
  
def callback_L(data):
  global lf_vel, lfon, f_vel
  if np.absolute(data.wrench.force.x) > TH_XY:
    lf_vel[0] = data.wrench.force.x * f_vel
  else:
    lf_vel[0] =0
  if np.absolute(data.wrench.force.y) > TH_XY:
    lf_vel[1] = data.wrench.force.y * f_vel
  else:
    lf_vel[1] =0
  if data.wrench.force.z > TH_Z:
    lfon = True
  else:
    lfon = False
  
def pubhumanpose():
  global rf_vel, lf_vel, commode, pub_val_list, pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_list
  init_foot_width = 0.1
  
  if rfon==True and lfon==True:
    commode = "CENTER"
  elif rfon==True and lfon==False:
    commode = "RIGHT"
  elif rfon==False and lfon==True:
    commode = "LEFT"
  else:
    commode = "CENTER"
    
  print "com" + commode
  
  if commode == "CENTER":
    pub_val_com.point.x = ( pub_val_rf.point.x + pub_val_lf.point.x ) / 2
    pub_val_com.point.y = ( pub_val_rf.point.y-init_foot_width + pub_val_lf.point.y+init_foot_width ) / 2
  elif commode == "RIGHT":
    pub_val_lf.point.x += lf_vel[0] / HZ
    pub_val_lf.point.y += lf_vel[1] / HZ
    if   pub_val_lf.point.x > pub_val_rf.point.x + 0.15: pub_val_lf.point.x = pub_val_rf.point.x + 0.15
    elif pub_val_lf.point.x < pub_val_rf.point.x - 0.10: pub_val_lf.point.x = pub_val_rf.point.x - 0.10
    if   pub_val_lf.point.y > pub_val_rf.point.y + 0.10: pub_val_lf.point.y = pub_val_rf.point.y + 0.10
    elif pub_val_lf.point.y < pub_val_rf.point.y - 0.05: pub_val_lf.point.y = pub_val_rf.point.y - 0.05
    pub_val_com.point.x = pub_val_rf.point.x
    pub_val_com.point.y = pub_val_rf.point.y-init_foot_width
  elif commode == "LEFT":
    pub_val_rf.point.x += rf_vel[0] / HZ
    pub_val_rf.point.y += rf_vel[1] / HZ
    if   pub_val_rf.point.x > pub_val_lf.point.x + 0.15: pub_val_rf.point.x = pub_val_lf.point.x + 0.15
    elif pub_val_rf.point.x < pub_val_lf.point.x - 0.10: pub_val_rf.point.x = pub_val_lf.point.x - 0.10
    if   pub_val_rf.point.y < pub_val_lf.point.y - 0.10: pub_val_rf.point.y = pub_val_lf.point.y - 0.10
    elif pub_val_rf.point.y > pub_val_lf.point.y + 0.05: pub_val_rf.point.y = pub_val_lf.point.y + 0.05
    pub_val_com.point.x = pub_val_lf.point.x
    pub_val_com.point.y = pub_val_lf.point.y+init_foot_width
  else:
    pub_val_com.point.x = ( pub_val_rf.point.x + pub_val_lf.point.x ) / 2
    pub_val_com.point.y = ( pub_val_rf.point.y-init_foot_width + pub_val_lf.point.y+init_foot_width ) / 2
    
  for i in range(len(pub_list)):
    pub_list[i].publish(pub_val_list[i])


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  sub_rfp = rospy.Subscriber("/human_tracker_rfw_ref", WrenchStamped, callback_R)
  sub_lfp = rospy.Subscriber("/human_tracker_lfw_ref", WrenchStamped, callback_L)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PointStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PointStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PointStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PointStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PointStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh]
  
  rospy.init_node('humansync_joy_publisher', anonymous=True)
  r = rospy.Rate(HZ)
  
  print "start ROS pub loop"
  while not rospy.is_shutdown():
    pubhumanpose()
    r.sleep()
