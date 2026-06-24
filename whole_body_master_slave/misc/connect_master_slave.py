#!/usr/bin/env python  
import math
import os
import roslib
import rospy
import time
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from multiprocessing import Value, Array, Process

p_tgts = ["com","lleg","rleg","larm","rarm"]
w_tgts = ["lleg","rleg","larm","rarm"]
#w_tgts_short = ["lf","rf","lh","rh"]
w_dof = 6 # wrench DOF = force3 + torque3
p_dof = 7 # pose DOF = pos3 + quartanion4
#master_host = 'http://tablis:11311'
master_host = 'http://localhost:11311'
master_origin = "BASE_LINK"
slave_host = 'http://jaxonred:11311'
slave_origin = "BODY"

RATE = 500

def sub_masterTgtPoses_from_master(pw):
    os.environ['ROS_MASTER_URI'] = master_host
    rospy.init_node("sub_masterTgtPoses_from_master", anonymous=True)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(p_tgts)):
            rospy.Subscriber( "master_"+p_tgts[i]+"_pose_out", PoseStamped, pose_CB, callback_args=i)
        rospy.spin()

def pose_CB(data, i):
    global pw
    pw[i*p_dof+0] = data.pose.position.x
    pw[i*p_dof+1] = data.pose.position.y
    pw[i*p_dof+2] = data.pose.position.z
    pw[i*p_dof+3] = data.pose.orientation.x
    pw[i*p_dof+4] = data.pose.orientation.y
    pw[i*p_dof+5] = data.pose.orientation.z
    pw[i*p_dof+6] = data.pose.orientation.w

        
def pub_masterTgtPoses_to_slave(pw):
    os.environ['ROS_MASTER_URI'] = slave_host
    rospy.init_node("pub_masterTgtPoses_to_slave", anonymous=True)
    pubs = [rospy.Publisher("master_"+p+"_pose_in", PoseStamped, queue_size=1) for p in p_tgts]
    rate = rospy.Rate(RATE)
    vals = [PoseStamped()] * len(p_tgts)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(0,len(vals)):
            vals[i].header.frame_id = "BODY"
            vals[i].header.stamp = rospy.Time.now()
            vals[i].pose.position.x    = pw[i*p_dof+0]
            vals[i].pose.position.y    = pw[i*p_dof+1]
            vals[i].pose.position.z    = pw[i*p_dof+2]
            vals[i].pose.orientation.x = pw[i*p_dof+3]
            vals[i].pose.orientation.y = pw[i*p_dof+4]
            vals[i].pose.orientation.z = pw[i*p_dof+5]
            vals[i].pose.orientation.w = pw[i*p_dof+6]
            pubs[i].publish(vals[i])
            infostr += p_tgts[i] + "\033[92m OK \033[0m"+str(pw[i*p_dof:i*p_dof+6])+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "->(pose)->Slave "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)


def sub_slaveEEWrench_from_slave(wl):
    os.environ['ROS_MASTER_URI'] = slave_host
    rospy.init_node("sub_slaveEEWrench_from_slave", anonymous=True)
    for i in range(len(w_tgts)):
        rospy.Subscriber( "slave_"+w_tgts[i]+"_wrench_out", WrenchStamped, callback, callback_args=i)
    rospy.spin()
    
def callback(data, id):
    global ww
    ww[id*w_dof+0] = data.wrench.force.x
    ww[id*w_dof+1] = data.wrench.force.y
    ww[id*w_dof+2] = data.wrench.force.z
    ww[id*w_dof+3] = data.wrench.torque.x
    ww[id*w_dof+4] = data.wrench.torque.y
    ww[id*w_dof+5] = data.wrench.torque.z


def pub_slaveEEWrench_to_master(ww):
    os.environ['ROS_MASTER_URI'] = master_host
    rospy.init_node("pub_slaveEEWrench_to_master", anonymous=True)
    pubs = [rospy.Publisher("slave_"+w+"_wrench_in",WrenchStamped, queue_size=1) for w in w_tgts]
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        tm_start = time.time()
        infostr = ""
        for i in range(len(pubs)):
            val = WrenchStamped()
            val.header.frame_id = master_origin
            val.wrench.force.x  = ww[i*w_dof+0]
            val.wrench.force.y  = ww[i*w_dof+1]
            val.wrench.force.z  = ww[i*w_dof+2]
            val.wrench.torque.x = ww[i*w_dof+3]
            val.wrench.torque.y = ww[i*w_dof+4]
            val.wrench.torque.z = ww[i*w_dof+5]
            pubs[i].publish(val)
            infostr += w_tgts[i] + "\033[92m OK \033[0m"+str(ww[i*w_dof:i*w_dof+6])+"\n"
        rate.sleep()
        rospy.loginfo_throttle(2, "Master<-(wrench)<- "+str(int(1.0/(time.time()-tm_start)))+" [fps]\n"+infostr)


if __name__ == '__main__':
    pw = Array('d', p_dof * len(p_tgts)) # world pose
    ww = Array('d', w_dof * len(w_tgts)) # world wrench

    processes = []
    processes += [Process(target=sub_masterTgtPoses_from_master, args=[pw])]
    processes += [Process(target=pub_masterTgtPoses_to_slave,    args=[pw])]
    processes += [Process(target=sub_slaveEEWrench_from_slave,   args=[ww])]
    processes += [Process(target=pub_slaveEEWrench_to_master,    args=[ww])]
    
    for p in processes:
        p.start()
    print("all process started")

    ### working ###

    for p in processes:
        p.join()
    print("all process finished")
