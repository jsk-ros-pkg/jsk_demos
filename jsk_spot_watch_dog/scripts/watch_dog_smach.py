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
from sound_play.libsoundplay import SoundClient
import smach
import smach_ros
import tf
import time
import copy


class Robot():
    def __init__(self):

        self.bridge = CvBridge()

        self.luminance = 0
        self.start_time = 0
        self.body_euler = [0,0,0]
        self.yaw_offset = 0
        self.battery_temp = 0;
        self.human_rects = [];

        self.luminance_threshold = rospy.get_param('~luminance_threshold')
        self.rest_time_per_hour = rospy.get_param('~rest_time_per_hour')
        self.human_width_threshold = rospy.get_param('~human_width_threshold')
        self.human_height_threshold = rospy.get_param('~human_height_threshold')
        self.max_tracking_yaw = rospy.get_param('~max_tracking_yaw')

        self.sub_image = rospy.Subscriber(
            rospy.resolve_name('~input'),
            CompressedImage, self.image_cb, queue_size=1, buff_size=2**26)

        try:
            msg_panorama_image = rospy.wait_for_message(
                rospy.resolve_name('~panorama_image'), Image, 10)
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('{}'.format(e))
            sys.exit(1)
        self.paranoma_width = msg_panorama_image.width
        self.paranoma_height = msg_panorama_image.height

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

        # hade-coding parameter due to the limitation of SPOT
        self.lookup_angle = rospy.get_param('~lookup_angle', -0.4) # radian
        self.change_body_duration = rospy.get_param('~change_body_duration', 1.0) # s
        self.temp_threshold = rospy.get_param('~temp_threshold', 50)
        #self.change_t = 0

        self.bark_sound = rospy.get_param('~bark_sound', '/home/spot/sound_play/bark.wav')
        self.sound_client = SoundClient(sound_action='robotsound', sound_topic='robotsound')

        self.claim_srv_name = "/spot/claim"
        self.sit_srv_name = "/spot/sit"
        self.stand_srv_name = "/spot/stand"
        self.poweron_srv_name = "/spot/power_on"
        self.poweroff_srv_name = "/spot/power_off"

        # claim control
        rospy.wait_for_service(self.claim_srv_name)
        try:
            claim_srv_call = rospy.ServiceProxy(self.claim_srv_name, Trigger)
            claim_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def stand_action(self):
        # servo on
        rospy.wait_for_service(self.poweron_srv_name)
        try:
            stand_srv_call = rospy.ServiceProxy(self.poweron_srv_name, Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service(self.stand_srv_name)
        try:
            stand_srv_call = rospy.ServiceProxy(self.stand_srv_name, Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def sit_action(self):

        # sit down
        rospy.wait_for_service(self.sit_srv_name)
        try:
            stand_srv_call = rospy.ServiceProxy(self.sit_srv_name, Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # servo off
        rospy.wait_for_service(self.poweroff_srv_name)
        try:
            poweroff_srv_call = rospy.ServiceProxy(self.poweroff_srv_name, Trigger)
            poweroff_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def body_action(self, euler):

        if not euler[0] == 0 or not euler[1] == 0 or not euler[2] == 0:
            self.sound_client.playWave(self.bark_sound)

        q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2]) # RPY
        pose = Pose()
        pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        self.body_pose_pub.publish(pose)

        rospy.wait_for_service(self.stand_srv_name)
        try:
            stand_srv_call = rospy.ServiceProxy(self.stand_srv_name, Trigger)
            stand_srv_call()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def image_cb(self, msg):

        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        ave_bright = np.mean(img)
        rospy.logdebug("the average britghness of {} is {}".format(rospy.resolve_name('~input'), ave_bright))
        # TODO: thread lock
        self.luminance = ave_bright

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        # TODO: thread lock
        self.body_euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

    def bat_cb(self, msg):
        # TODO: thread lock
        self.battery_temp = np.max(np.array(msg.battery_states[0].temperatures))

    def detection_cb(self, msg):
        # TODO: thread lock
        self.human_rects = msg.rects

class DayMotion(smach.State):

    def __init__(self, robot, outcomes=[], input_keys = [], output_keys = [], io_keys = []):
        outcomes.extend(['light_off', 'preempted'])

        super(DayMotion, self).__init__(outcomes, input_keys, output_keys, io_keys)

        self.robot = robot
        self.luminance_thresh  = self.robot.luminance_threshold

    def execute(self, userdata):

        while self.robot.luminance >= self.luminance_thresh:
            rospy.logdebug("day motion, luminance: {}, stand duration: {:3f} min".format(self.robot.luminance, (time.time() - self.robot.start_time) / 60))
            if rospy.is_shutdown():
                return 'preempted'

            outcome = self.specific_execute(userdata)
            if not outcome is None:
                return outcome

        self.robot.sit_action() # sit
        rospy.loginfo("sit down because surrounding is dark")
        return 'light_off'

    def specific_execute(self, userdata):
        time.sleep(1)
        return None

class Rest(DayMotion):
    def __init__(self, robot):
        super(Rest, self).__init__(robot, outcomes = ['stand'])

    def specific_execute(self, userdata):
        time.sleep(1.0)

        if (time.time() - self.robot.start_time) / 60 > self.robot.rest_time_per_hour:
                rospy.loginfo("finish rest")
                self.robot.start_time = time.time()
                self.robot.stand_action()
                return 'stand'

        return None

class Watch(DayMotion):
    def __init__(self, robot):
        super(Watch, self).__init__(robot, outcomes = ['sit'])

    def specific_execute(self, userdata):
        # battery temperature
        if self.robot.battery_temp > self.robot.temp_threshold:
            rospy.logwarn("the battery temperature is higher than the threshold: {} vs {}, sit down and turn off servo".format(self.robot.battery_temp, self.robot.temp_threshold))
            self.robot.start_time = time.time()
            self.robot.sit_action()
            return 'sit'

        # rest
        if  (time.time() - self.robot.start_time) / 60 > 60 - self.robot.rest_time_per_hour:
            rospy.loginfo("have a rest")
            self.robot.start_time = time.time()
            self.robot.sit_action()
            return 'sit'

        # watch person
        max_person_height = 0
        target_person = None

        # workaround to handle the smaller bbox height when robot look up (head up).
        human_height_threshold = self.robot.human_height_threshold
        if self.robot.body_euler[1] < self.robot.lookup_angle * 0.5:
            human_height_threshold = self.robot.human_height_threshold * 0.8

        human_rects = copy.copy(self.robot.human_rects) # copy
        for rect in human_rects:
            if rect.width < self.robot.human_width_threshold * rect.height and rect.height > human_height_threshold * self.robot.paranoma_height:
                if rect.height > max_person_height:
                    max_person_height = rect.height
                    target_person = rect

        if target_person is None:
            target_euler = [0, 0, 0]
            # only update yaw offset when the robot attitude is level
            if np.abs(self.robot.body_euler[0]) < 0.05 and np.abs(self.robot.body_euler[1]) < 0.05:
                self.robot.yaw_offset = self.robot.body_euler[2]
        else:
            rel_yaw_angle =  (self.robot.paranoma_width / 2 - (target_person.x + target_person.width/2)) / float(self.robot.paranoma_width) * 2 * np.pi

            if np.abs(rel_yaw_angle) < self.robot.max_tracking_yaw:
                target_euler = [0, self.robot.lookup_angle, rel_yaw_angle + self.robot.body_euler[2] - self.robot.yaw_offset]
            else:
                target_euler = [0, 0, 0]

        self.robot.body_action(target_euler)

        time.sleep(self.robot.change_body_duration)

        return None

class DarkMotion(smach.State):

    def __init__(self, robot):
        super(DarkMotion, self).__init__(outcomes = ['light_on', 'preempted'])

        self.robot = robot
        self.luminance_thresh  = self.robot.luminance_threshold

    def execute(self, userdata):

        rospy.loginfo("dark motion, luminance: {}, stand duration: {}".format(self.robot.luminance, time.time() - self.robot.start_time))

        # TODO: use threading.Event and callback function instead of while function (check MonitorState)
        # https://qiita.com/tag1216/items/2dcb112f8018eb19a999
        while self.robot.luminance < self.luminance_thresh:
            if rospy.is_shutdown():
                return 'preempted'
            time.sleep(1)

        self.robot.stand_action() # stand
        rospy.loginfo("stand up because surrounding is bright")
        self.robot.start_time = time.time()
        return 'light_on'

def main():
    rospy.init_node('watchdog_demo')

    simple_stand_sit_flag = rospy.get_param('~simple_stand_sit_flag', False)

    sm = smach.StateMachine(outcomes=['preempted'])
    #TODO: do we need to explicitly define input_keys, such as 'luminance' (we set these attribute in  __setattr_ in Robot.__init__())
    spot = Robot()

    # Open the container
    with sm:

        smach.StateMachine.add('dark_motion', DarkMotion(spot),
                               transitions={'light_on':'day_motion', 'preempted': 'preempted'})

        if simple_stand_sit_flag:

            smach.StateMachine.add('day_motion', DayMotion(spot),
                                   transitions={'light_off':'dark_motion',
                                                'preempted': 'preempted'})
        else:
            # Create the sub SMACH state machine for day motion
            sm_day_motion = smach.StateMachine(outcomes=['light_off', 'preempted'])

            # Open the container
            with sm_day_motion:

                smach.StateMachine.add('watch', Watch(spot),
                                       transitions={'light_off': 'light_off',
                                                    'sit':'rest',
                                                    'preempted':'preempted'})

                smach.StateMachine.add('rest', Rest(spot),
                                       transitions={'light_off': 'light_off',
                                                    'stand':'watch',
                                                    'preempted':'preempted'})


            smach.StateMachine.add('day_motion', sm_day_motion,
                                   transitions={'light_off':'dark_motion',
                                                'preempted':'preempted'})

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    ros.spin()
    sis.stop()

if __name__ == '__main__':
    main()
