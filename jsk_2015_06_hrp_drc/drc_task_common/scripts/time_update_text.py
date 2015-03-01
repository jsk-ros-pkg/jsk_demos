#!/usr/bin/env python

import rospy
from std_msgs.msg import Time, String

rospy.init_node("time_update_text")

def callback(msg):
    global pub
    now = rospy.Time.now()
    diff = now - msg.data
    text = "%s%s%s" % (prefix_text, diff.to_sec(), suffix_text)
    pub.publish(String(data=text))

prefix_text = rospy.get_param("~prefix", "")
suffix_text = rospy.get_param("~suffix", "")
pub = rospy.Publisher("~text", String)
sub = rospy.Subscriber("~time", Time, callback)

rospy.spin()
