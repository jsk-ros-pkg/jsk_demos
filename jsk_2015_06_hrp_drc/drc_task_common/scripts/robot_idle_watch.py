#!/usr/bin/env python

# watch if robot is idle or moving

import rospy
from threading import Lock
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from drc_com_common.msg import FC2OCSLarge
from std_msgs.msg import Int32
rospy.init_node("robot_idle_watch")

lock = Lock() # lock is required because subscribers run in subthreads
watch_controller = rospy.get_param("~watch_controller", "/fullbody_controller/joint_trajectory_action")
rate = rospy.get_param("~rate", 10)
is_robot_moving = False

def controllerCallback(msg):
    global lock, is_robot_moving
    with lock:
        for status in msg.status_list:
            # if status.status == GoalStatus.Active, 
            # robot is moving
            if status.status == GoalStatus.ACTIVE:
                is_robot_moving = True
                return
        # If the message does not have GoalStatus.Active, 
        # we should change status to IDLE
        is_robot_moving = False
pub = rospy.Publisher("robot_status", Int32)
sub = rospy.Subscriber(watch_controller + "/status", GoalStatusArray, controllerCallback)

r = rospy.Rate(rate)
while not rospy.is_shutdown():
    with lock:
        msg = Int32()
        if is_robot_moving:
            msg.data = FC2OCSLarge.ROBOT_MOVING
        else:
            msg.data = FC2OCSLarge.ROBOT_IDLE
        pub.publish(msg)
    r.sleep()

