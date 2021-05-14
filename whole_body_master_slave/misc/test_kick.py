#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import signal
import rospy
import tf
import math
from std_msgs.msg import *
from geometry_msgs.msg import *


if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal.SIG_DFL)
  pub_com = rospy.Publisher('/human_tracker_com_ref', PoseStamped, queue_size=10)
  pub_rf = rospy.Publisher('/human_tracker_rf_ref', PoseStamped, queue_size=10)
  pub_lf = rospy.Publisher('/human_tracker_lf_ref', PoseStamped, queue_size=10)
  pub_rh = rospy.Publisher('/human_tracker_rh_ref', PoseStamped, queue_size=10)
  pub_lh = rospy.Publisher('/human_tracker_lh_ref', PoseStamped, queue_size=10)
  pub_zmp = rospy.Publisher('/human_tracker_zmp_ref', PointStamped, queue_size=10)
#   pub_rfw = rospy.Publisher('/human_tracker_rfw_ref', WrenchStamped, queue_size=10)
#   pub_lfw = rospy.Publisher('/human_tracker_lfw_ref', WrenchStamped, queue_size=10)
  pub_list = [pub_com,pub_rf,pub_lf,pub_rh,pub_lh,pub_zmp]
  
  rospy.init_node('kick_publisher', anonymous=True)
  HZ = 100.0
  r = rospy.Rate(HZ)
  
  pub_val_com = PoseStamped()
  pub_val_rf = PoseStamped()
  pub_val_lf = PoseStamped()
  pub_val_rh = PoseStamped()
  pub_val_lh = PoseStamped()
  pub_val_zmp = PointStamped()
  pub_val_list = [pub_val_com, pub_val_rf, pub_val_lf, pub_val_rh, pub_val_lh, pub_val_zmp]
  
  print "start kick"
  loop = 0
  kick_loop = 0
  while not rospy.is_shutdown():
    
    if loop < 3 * HZ:
      pub_val_com.pose.position.x = 0.02;
      pub_val_com.pose.position.y = 0;
      pub_val_com.pose.position.z = 0;
      
    elif loop < 6 * HZ:
      pub_val_com.pose.position.y = 0.1;
      pub_val_rf.pose.position.z = 0.1;
      
    elif loop < 10 * HZ:
      pub_val_rf.pose.position.x = 0.2 * math.sin(2*math.pi*kick_loop/HZ * 1);
      kick_loop += 1
      
    elif loop < 12 * HZ:
      pub_val_com.pose.position.x = 0.02;
      pub_val_com.pose.position.y = 0;
      pub_val_com.pose.position.z = 0;
      pub_val_rf.pose.position.y = 0;
      pub_val_rf.pose.position.z = 0;
      
    else:
      break
    
    
    for i in range(len(pub_list)):
      pub_list[i].publish(pub_val_list[i])
    r.sleep()
    loop += 1
