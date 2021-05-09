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
    self.sub = {lr : rospy.Subscriber(lr+"_force_input", WrenchStamped, self.cb, lr) for lr in self.LR}
    self.pub = {lr : rospy.Publisher("/"+lr+"arm_marker/image_marker", ImageMarker2, queue_size=10) for lr in self.LR}
    rospy.loginfo("start main loop")
    rospy.spin()

  def cb(self, msg, lr):
    mrk                          = ImageMarker2()
    mrk.type                     = ImageMarker2.LINE_LIST3D
    mrk.header.stamp             = rospy.Time.now()
    mrk.lifetime                 = rospy.Time(0.1)
    mrk.points3D.header.frame_id = msg.header.frame_id
    mrk.points3D.points          = [Point(0,0,0), Point(msg.wrench.force.x/50, msg.wrench.force.y/50, msg.wrench.force.z/50)]
    raw_force_vec                = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    warn_level                   = np.clip(np.linalg.norm(raw_force_vec)/50 , 0, 1) # 20N = 100% warn = 1
    mrk.outline_colors           = [ColorRGBA(1, 1 - warn_level, 0, 0.5)]    
    self.pub[lr].publish(mrk)

if __name__ == '__main__':
  inst = DrawForceOnImage()
