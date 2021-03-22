#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('TF2PoseStamped')

    listener = tf.TransformListener()

    tgts = ["com","lf","rf","lh","rh","head"]
    pubs = [rospy.Publisher("/master_"+tgt+"_pose", PoseStamped, queue_size=10) for tgt in tgts]
    vals = [PoseStamped() for i in range(0, len(tgts))]
    rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(tgts)):
            try:
                (pos, rot) = listener.lookupTransform("/operator_origin", "/vive_"+tgts[i]+"_tf", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                infostr += tgts[i] + "\033[91m waiting... \033[0m\n"
                continue
            else:
                vals[i].header.frame_id = "operator_origin"
                vals[i].header.stamp = rospy.Time.now()
                vals[i].pose.position.x = pos[0]
                vals[i].pose.position.y = pos[1]
                vals[i].pose.position.z = pos[2]
                vals[i].pose.orientation.x = rot[0]
                vals[i].pose.orientation.y = rot[1]
                vals[i].pose.orientation.z = rot[2]
                vals[i].pose.orientation.w = rot[3]
                pubs[i].publish(vals[i])
                infostr += tgts[i] + "\033[92m OK \033[0m"+str(pos)+" "+str(rot)+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Working at "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)
