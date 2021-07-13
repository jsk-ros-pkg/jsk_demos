#!/usr/bin/env python
# -*- encoding: utf-8 -*-

"""this script is a demo that a robot play music"""

from geometry_msgs.msg import Quaternion
import math
import PyKDL
import rospy
import random
from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
import sys
import threading


def main():

    rospy.init_node('spot_sing_and_dance')
    node = SpotPlayAndDance()


class SpotPlayAndDance:

    def __init__(self):

        self.client = SoundClient(
            sound_action='/robotsound', sound_topic='/robotsound')
        self.spot_client = SpotRosClient()

        music_list = rospy.get_param('~music_list')
        music_name = rospy.get_param('~music_name',None)
        try:
            if music_name is None or music_name == '':
                music_name, music_entry = random.sample(music_list.items(), 1)[0]
            elif music_name not in music_list:
                rospy.logerr('{} is not in music_list'.format(music_name))
                sys.exit()
            else:
                music_entry = music_list[music_name]
            play_file = music_entry['filepath']
            self.bpm = float(music_entry['bpm'])
        except Exception as e:
            rospy.logerr('Failed to load music: {}'.format(e))
            sys.exit()

        self.do_dance = True
        self.thread_dance = threading.Thread(target=self.dance)
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo('Started to play {} from {}'.format(music_name, play_file))
        self.thread_dance.start()
        self.client.playWave(play_file, blocking=True)

    def dance(self):

        rot_z_plus = PyKDL.Rotation.RotZ(math.pi/10)
        rot_z_minus = PyKDL.Rotation.RotZ(-math.pi/10)
        rot_y = PyKDL.Rotation.RotY(-math.pi/12)

        pose_left = (rot_y * rot_z_plus).GetQuaternion()
        pose_right = (rot_y * rot_z_minus).GetQuaternion()

        pose_num = 0
        loop_duration = rospy.Duration((60.0 / self.bpm) * 2)
        time_desired = rospy.Time.now() + loop_duration
        while self.do_dance and not rospy.is_shutdown():
            if self.client.actionclient.wait_for_result(rospy.Duration(0.1)):
                break
            if pose_num == 0:
                self.spot_client.pubBodyPose(-0.1, Quaternion(w=1.0))
                self.spot_client.stand()
            elif pose_num == 1:
                self.spot_client.pubBodyPose(0, Quaternion(
                    x=pose_left[0], y=pose_left[1], z=pose_left[2], w=pose_left[3]))
                self.spot_client.stand()
            elif pose_num == 2:
                self.spot_client.pubBodyPose(-0.1, Quaternion(w=1.0))
                self.spot_client.stand()
            elif pose_num == 3:
                self.spot_client.pubBodyPose(0, Quaternion(
                    x=pose_right[0], y=pose_right[1], z=pose_right[2], w=pose_right[3]))
                self.spot_client.stand()
            pose_num += 1
            if pose_num > 3:
                pose_num = 0
            rospy.sleep(time_desired - rospy.Time.now())
            time_desired += loop_duration

    def shutdown_hook(self):
        rospy.logwarn('stop playing music...')
        self.client.stopAll()
        self.do_dance = False
        self.thread_dance.join()
        self.spot_client.pubBodyPose(0.0, Quaternion(w=1.0))
        self.spot_client.stand()


if __name__ == '__main__':
    main()
