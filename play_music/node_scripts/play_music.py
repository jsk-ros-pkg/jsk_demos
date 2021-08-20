#!/usr/bin/env python
# -*- encoding: utf-8 -*-

"""this script is a demo that a robot play music"""

from sound_play.libsoundplay import SoundClient
import rospy
import os
import random


def main():

    rospy.init_node('play_music')

    music_directory = rospy.get_param('~music_directory')

    music_files = []
    for music_file in os.listdir(music_directory):
        if music_file.split('.')[-1] == 'mp3' or music_file.split('.')[-1] == 'wav':
            music_files.append(music_file)

    if len(music_files) == 0:
        rospy.logerr('No music file found in {}'.format(music_directory))
        return

    play_file = music_directory + '/' + random.sample(music_files, 1)[0]

    client = SoundClient(sound_action='/robotsound', sound_topic='/robotsound')

    rospy.sleep(3)

    rospy.loginfo('Started to play {}'.format(play_file))

    client.playWave(play_file, blocking=True)


if __name__ == '__main__':
    main()
