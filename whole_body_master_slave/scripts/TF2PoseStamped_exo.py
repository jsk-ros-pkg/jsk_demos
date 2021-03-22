#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import time
from geometry_msgs.msg import PoseStamped

from multiprocessing import Value, Array, Process
import os

prefixes = ["com","lf","rf","lh","rh"]
tgts = ["BASE_LINK","LLEG_LINK5","RLEG_LINK5","LARM_LINK6","RARM_LINK6"]
p_dof = 7 # pos3 + quartanion4

def process1(ja):
    os.environ['ROS_MASTER_URI'] = 'http://jaxonred:11311'
    rospy.init_node('pose_pub_to_jaxon', anonymous=True)
    pubs = [rospy.Publisher("/human_tracker_"+p+"_ref", PoseStamped, queue_size=1) for p in prefixes]
    rate = rospy.Rate(200)
    vals = [PoseStamped() for i in range(0, len(tgts))]
    while not rospy.is_shutdown():
        for i in range(0,len(vals)):
            vals[i].header.frame_id = "BODY"
            vals[i].header.stamp = rospy.Time.now()
            vals[i].pose.position.x    = ja[i*p_dof+0]
            vals[i].pose.position.y    = ja[i*p_dof+1]
            vals[i].pose.position.z    = ja[i*p_dof+2]
            vals[i].pose.orientation.x = ja[i*p_dof+3]
            vals[i].pose.orientation.y = ja[i*p_dof+4]
            vals[i].pose.orientation.z = ja[i*p_dof+5]
            vals[i].pose.orientation.w = ja[i*p_dof+6]
            pubs[i].publish(vals[i])            
        rate.sleep()

def process2(ja):
    os.environ['ROS_MASTER_URI'] = 'http://tablis:11311'
    rospy.init_node('tf_sub_from_tablis', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(0,len(tgts)):
            try:
                (pos, rot) = listener.lookupTransform("/BASE_LINK", "/"+tgts[i], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                infostr += tgts[i] + "\033[91m waiting... \033[0m\n"
                continue
            else:
                ja[i*p_dof+0 : i*p_dof+3] = pos[:]
                ja[i*p_dof+3 : i*p_dof+7] = rot[:]
                infostr += tgts[i] + "\033[92m OK \033[0m"+str(pos)+" "+str(rot)+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Working at "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)

if __name__ == '__main__':
    ja = Array('d', len(tgts) * p_dof)
    process1 = Process(target=process1, args=[ja])
    process2 = Process(target=process2, args=[ja])
    process1.start()
    process2.start()
    ### working ###
    process1.join()
    process2.join()
    print("process ended")
