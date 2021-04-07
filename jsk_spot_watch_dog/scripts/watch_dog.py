#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Quaternion
from jsk_recognition_msgs.msg import RectArray
import numpy as np
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import CompressedImage, Image
from spot_msgs.msg import BatteryStateArray
from std_srvs.srv import Trigger
import tf
import time

class WatchDog():

    def __init__(self):
        self.bridge = CvBridge()

        self.brightness_threshold = rospy.get_param('~brightness_threshold')
        self.rest_time_per_hour = rospy.get_param('~rest_time_per_hour')

        self.human_width_threshold = rospy.get_param('~human_width_threshold')
        self.human_height_threshold = rospy.get_param('~human_height_threshold')

        self.max_tracking_yaw = rospy.get_param('~max_tracking_yaw')

        self.sub_image = rospy.Subscriber(
            rospy.resolve_name('~input'),
            CompressedImage, self.image_cb, queue_size=1, buff_size=2**26)

        # workaround to get the image size of panorama image
        self.sub_panorama = rospy.Subscriber(
            rospy.resolve_name('~panorama'),
            Image, self.panorama_cb, queue_size=1)

        self.sub_detection = rospy.Subscriber(
            rospy.resolve_name('~human'),
            RectArray, self.detection_cb, queue_size=1)

        self.sub_odom = rospy.Subscriber(
            '/spot/odometry',
            Odometry, self.odom_cb, queue_size=1)

        self.sub_bat = rospy.Subscriber(
            '/spot/status/battery_states',
            BatteryStateArray, self.bat_cb, queue_size=1)

        self.body_pose_pub = rospy.Publisher('/spot/body_pose', Pose, queue_size=1)


        self.idle = 0
        self.stand = 1
        self.rest = 2
        self.status = self.idle

        self.body_euler = [0,0,0]
        self.yaw_offset = 0

        self.start_t = 0

        self.paranoma_width = None
        self.paranoma_height = None

        # hade-coding parameter due to the limitation of SPOT
        self.lookup_angle = -0.4 # radian
        self.change_body_duration = 1.0 # s
        self.change_t = 0
        self.temp_threshold = 50
        self.battery_temp = 0;

    def _body_action(self, euler):
        if time.time() - self.change_t < self.change_body_duration:
            return

        self.change_t = time.time()

        q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2]) #RPY
        pose = Pose()
        pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        self.body_pose_pub.publish(pose)

        rospy.wait_for_service('/spot/stand')
        try:
            stand_srv_call = rospy.ServiceProxy('/spot/stand', Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def _sit_action(self):
        rospy.wait_for_service('/spot/sit')
        try:
            stand_srv_call = rospy.ServiceProxy('/spot/sit', Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # servo off
        rospy.wait_for_service('/spot/power_off')
        try:
            stand_srv_call = rospy.ServiceProxy('/spot/power_off', Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.start_t = time.time()

    def _stand_action(self):
        # servo on
        rospy.wait_for_service('/spot/power_on')
        try:
            stand_srv_call = rospy.ServiceProxy('/spot/power_on', Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/spot/stand')
        try:
            stand_srv_call = rospy.ServiceProxy('/spot/stand', Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.start_t = time.time()

    # workaround to get image size
    def panorama_cb(self, msg):

        self.paranoma_width = msg.width
        self.paranoma_height = msg.height

        self.sub_panorama.unregister()

    def image_cb(self, msg):

        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        ave_bright = np.mean(img)
        rospy.logdebug("the average britghness of {} is {}, status is {}".format(rospy.resolve_name('~input'), ave_bright, self.status))
        if self.status == self.idle and  ave_bright > self.brightness_threshold:
            rospy.loginfo("stand up because surrounding is bright")
            self._stand_action()
            self.status = self.stand

        if self.status > self.idle:

            if self.status == self.stand and (time.time() - self.start_t) / 60 > 60 - self.rest_time_per_hour:
                rospy.loginfo("have a rest")
                self._sit_action()
                self.status = self.rest

            if self.status == self.rest and (time.time() - self.start_t) / 60  > self.rest_time_per_hour:
                rospy.loginfo("finish rest")
                self._stand_action()
                self.status = self.stand

            if ave_bright < self.brightness_threshold:
                rospy.loginfo("sit down because surrounding is dark")
                self._sit_action()
                self.status = self.idle

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.body_euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

    def bat_cb(self, msg):
        self.battery_temp = np.max(np.array(msg.battery_states[0].temperatures))
        #print(self.battery_temp)

        if self.status == self.stand and self.battery_temp > self.temp_threshold:
            rospy.logwarn("the battery temperature is higher than the threshold: {} vs {}, sit down and turn off servo".format(self.battery_temp, self.temp_threshold))
            self._sit_action()
            self.status = self.rest

    def detection_cb(self, msg):

        if self.paranoma_height is None:
            return

        max_person_height = 0
        target_person = None

        debug_max_height = 0
        debug_bbox = None

        # workaround to handle the smaller bbox height when robot look up (head up).
        human_height_threshold = self.human_height_threshold
        if self.body_euler[1] < self.lookup_angle * 0.5:
            human_height_threshold = self.human_height_threshold * 0.8

        for rect in msg.rects:

            if rect.height > debug_max_height and rect.width < self.human_width_threshold * rect.height:
                debug_max_height = rect.height
                debug_bbox = rect

            if rect.width < self.human_width_threshold * rect.height and rect.height > human_height_threshold * self.paranoma_height:
                if rect.height > max_person_height:
                    max_person_height = rect.height
                    target_person = rect

                #rospy.loginfo("this is a closed human: [{}, {}, {}, {}]".format(rect.x, rect.y, rect.width, rect.height))

        # if debug_bbox is not None:
        #     rospy.loginfo("max bbox: {}, {}".format(debug_bbox.width / float(debug_bbox.height), debug_bbox.height / float(self.paranoma_height)))

        if self.status == self.stand:
            if target_person is None:
                target_euler = [0, 0, 0]
                self.yaw_offset = self.body_euler[2]
            else:
                rel_yaw_angle =  (self.paranoma_width / 2 - (target_person.x + target_person.width/2)) / float(self.paranoma_width) * 2 * np.pi

                if np.abs(rel_yaw_angle) < self.max_tracking_yaw:
                    target_euler = [0, self.lookup_angle, rel_yaw_angle + self.body_euler[2] - self.yaw_offset]
                else:
                    target_euler = [0, 0, 0]

            #print(target_euler, self.body_euler)

            self._body_action(target_euler)

if __name__ == '__main__':
    rospy.init_node('spot_watchdog')
    wath_action = WatchDog()
    rospy.spin()
