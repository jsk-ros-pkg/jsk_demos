#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from image_view2.msg import ImageMarker2
import numpy as np

class DrawForceOnImage():
  LR   = ["l", "r"]
  sgns = { "l" : 1, "r" : -1 }

  def __init__(self):
    rospy.init_node('draw_force_on_image')

    # self.sub_l = rospy.Subscriber("/left_endeffector/wrench", WrenchStamped, self.cb_l)
    # self.sub_r = rospy.Subscriber("/right_endeffector/wrench", WrenchStamped, self.cb_r)
    # self.pub   = rospy.Publisher('image_marker', ImageMarker2, queue_size=10)
    self.sub_l = rospy.Subscriber("/dual_panda/larm_state_controller/F_ext", WrenchStamped, self.cb_l)
    self.sub_r = rospy.Subscriber("/dual_panda/rarm_state_controller/F_ext", WrenchStamped, self.cb_r)
    # self.pub   = rospy.Publisher('image_marker', ImageMarker2, queue_size=10)
    self.pub_l   = rospy.Publisher('/larm_marker/image_marker', ImageMarker2, queue_size=10)
    self.pub_r   = rospy.Publisher('/rarm_marker/image_marker', ImageMarker2, queue_size=10)

    rospy.loginfo("start main loop")
    rospy.spin()

  def cb_l(self, msg):
    self.pub_force("l", msg.wrench)
    
  def cb_r(self, msg):
    self.pub_force("r", msg.wrench)
    
  def pub_force(self, lr, wrench):
    mrk                          = ImageMarker2()
    mrk.type                     = ImageMarker2.LINE_LIST3D
    mrk.header.stamp             = rospy.Time.now()
    mrk.lifetime                 = rospy.Time(0.1)
    # mrk.points3D.header.frame_id = lr + "_gripper_tool_frame"
    mrk.points3D.header.frame_id = lr + "arm_K"
    mrk.points3D.points          = [Point(0,0,0), Point(wrench.force.x/50, wrench.force.y/50, wrench.force.z/50)]
    raw_force_vec                = np.array([wrench.force.x, wrench.force.y, wrench.force.z])
    warn_level                   = np.clip(np.linalg.norm(raw_force_vec)/50 , 0, 1) # 20N = 100% warn = 1
    mrk.outline_colors           = [ColorRGBA(1, 1 - warn_level, 0, 0.5)]    
    self.pub.publish(mrk)    

    if lr == "l":
      self.pub_l.publish(mrk)
    else:
      self.pub_r.publish(mrk)    

if __name__ == '__main__':
  inst = DrawForceOnImage()
