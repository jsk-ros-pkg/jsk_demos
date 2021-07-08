#!/usr/bin/env python
# -*- encoding: utf-8 -*-

"""this script is a demo that a robot play music"""

from sound_play.libsoundplay import SoundClient
from spot_ros_client.libspotros import SpotRosClient
import PyKDL
import rospy
import os
import random
import sys
import math
from geometry_msgs.msg import Quaternion
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestGoal
import threading

def main():

    rospy.init_node('play_music')
    node = SpotPlayAndDance()


class SpotPlayAndDance:

    def dance(self):

        rot_z_plus = PyKDL.Rotation.RotZ(math.pi/10)
        rot_z_minus = PyKDL.Rotation.RotZ(-math.pi/10)
        rot_y = PyKDL.Rotation.RotY(-math.pi/12)

        pose_left = ( rot_y * rot_z_plus ).GetQuaternion()
        pose_right = ( rot_y * rot_z_minus ).GetQuaternion()

        pose_num = 0
        loop_duration = rospy.Duration(( 60.0 / self.bpm ) * 2)
        time_desired = rospy.Time.now() + loop_duration
        while self.do_dance and not rospy.is_shutdown():
            if self.client.actionclient.wait_for_result(rospy.Duration(0.1)):
                break
            if pose_num == 0:
                self.spot_client.pubBodyPose(-0.1,Quaternion(w=1.0))
                self.spot_client.stand()
            elif pose_num == 1:
                self.spot_client.pubBodyPose(0,Quaternion(x=pose_left[0],y=pose_left[1],z=pose_left[2],w=pose_left[3]))
                self.spot_client.stand()
            elif pose_num == 2:
                self.spot_client.pubBodyPose(-0.1,Quaternion(w=1.0))
                self.spot_client.stand()
            elif pose_num == 3:
                self.spot_client.pubBodyPose(0,Quaternion(x=pose_right[0],y=pose_right[1],z=pose_right[2],w=pose_right[3]))
                self.spot_client.stand()
            pose_num += 1
            if pose_num > 3:
                pose_num = 0
            rospy.sleep( time_desired - rospy.Time.now() )
            time_desired += loop_duration

    def shutdown_hook(self):
        rospy.logwarn('stop playing music...')
        self.client.stopAll()
        self.do_dance = False
        self.thread_dance.join()
        self.spot_client.pubBodyPose(0.0,Quaternion(w=1.0))
        self.spot_client.stand()

    def __init__(self):

        self.client = SoundClient(sound_action='/robotsound', sound_topic='/robotsound')
        self.spot_client = SpotRosClient()

        music_directory = rospy.get_param('~music_directory')
        music_files = []
        for music_file in os.listdir(music_directory):
            if music_file.split('.')[-1] == 'mp3' or music_file.split('.')[-1] == 'wav':
                music_files.append(music_file)

        if len(music_files) == 0:
            rospy.logerr('No music file found in {}'.format(music_directory))
            sys.exit(1)

        play_file = music_directory + '/' + random.sample(music_files, 1)[0]
        # TODO calc bpm
        self.bpm = 130

        self.do_dance = True
        self.thread_dance = threading.Thread(target=self.dance)
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo('Started to play {}'.format(play_file))
        self.thread_dance.start()
        self.client.playWave(play_file, blocking=True)


if __name__ == '__main__':
    main()
