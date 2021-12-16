#!/usr/bin/env python

import rospy
import roslaunch
import actionlib
import rospkg

from posedetection_msgs.msg import ObjectDetection


class TrashLift():
    def __init__(self):
        rospy.init_node("trash_wait_node")
        rospy.logdebug('__init__called')

        self.subscriber_object_detection = None
        self.trash_wait = None

    def trashmark_callback(self, msg):
        print "hoge"

    def run_initial(self):
        # launch recognition of trashmark
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('jsk_spot_take_out_trash') +\
            '/launch/trash_mark.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
            )
        self.roslaunch_parent.start()

        # self.trash_wait = rospy.wait_for_message(
        #     '/kinova_wrist_camera/color/ObjectDetection',
        #     ObjectDetection,
        #     )
        self.subscriber_object_detection = rospy.Subscriber(
            '/head_camera/rgb/ObjectDetection',
            # '/kinova_wrist_camera/color/ObjectDetection',
            ObjectDetection,
            self.trashmark_callback)
        rospy.spin()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('target detected')

    def run_final(self, start_node, end_node):

        rospy.logdebug('run_final() called')
        self.roslaunch_parent.shutdown()


if __name__ == '__main__':
    trash = TrashLift()
    trash.run_initial()
