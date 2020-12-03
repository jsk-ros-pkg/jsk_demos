#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

class PickNearestBoundingBox():

  def __init__(self):
    rospy.init_node("PickNearestBoundingBox", anonymous=False)
    self.c_sub = rospy.Subscriber("~input_point", PointStamped, self.click_cb)
    self.b_sub = rospy.Subscriber("~input_boxes", BoundingBoxArray, self.boxes_cb)
    self.b_pub = rospy.Publisher("~output_box", BoundingBox, queue_size=1)
    self.p_pub = rospy.Publisher("~output_pose", PoseStamped, queue_size=1)
    self.tfl   = tf.TransformListener()
    self.bba = None
    rospy.spin()

  def click_cb(self, msg):
    stamp = rospy.Time.now()
    try:
      self.tfl.waitForTransform("/base_link", "/ray_target", stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/base_link", "/ray_target", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return

    if self.bba:
      candidates = []
      for bb in self.bba.boxes:
        if bb.dimensions.x * bb.dimensions.y * bb.dimensions.z > 0.0:
          candidates += [(pow(pos[0] - bb.pose.position.x, 2) + pow(pos[1] - bb.pose.position.y, 2) + pow(pos[2] - bb.pose.position.z, 2), bb.pose)]
      out_msg = PoseStamped()
      out_msg.header = bb.header
      out_msg.pose.position = min(candidates)[1].position
      self.p_pub.publish(out_msg)
      
    else:
      rospy.logwarn("input_point is given, but input_boxes has not been received...")

  def boxes_cb(self, msg):
    self.bba = msg

    
if __name__ == '__main__':
  inst = PickNearestBoundingBox()
