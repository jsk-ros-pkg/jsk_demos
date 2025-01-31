#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt16, String
from ros_speak import play_sound
from pathlib import Path  # pathlibをインポート
from dynamic_reconfigure.server import Server
from kxr_controller.cfg import PoohScenarioConfig
import random

class Eyebrows(object):
    EMPTY = ""
    HAPPY = "happy"
    SAD = "sad"
    NORMAL = "normal"
    SURPRISED = "surprised"
    ANGRY = "angry"

class Necks(object):
    EMPTY = ""
    TILT = "tilt"
    NOD = "nod"
    NATURAL_NOD = "natural_nod"
    DISAGREE = "disagree"

class Arms(object):
    EMPTY = ""
    RIGHT_HAND_UP = "right_hand_up"
    LEFT_HAND_CHIN = "left_hand_chin"
    RIGHT_HAND_MOUTH = "right_hand_mouth"
    CROSS_ARMS = "cross_arms"
    BANZAI = "banzai"
    ONEGAI = "onegai"
    LEFT_HAND_POINT = "left_hand_point"
    RIGHT_HAND_BYE = "right_hand_bye"
    LEFT_HAND_BYE = "left_hand_bye"
    LEFT_HAND_UP = "left_hand_up"
    START_SHAKE = "start_shake"
    JANKEN = "janken"
    AIKO = "aiko"
    INIT_POSE = "init_pose"
    
action_motion_pair = {
    (0, 1, 1): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_UP),
    (0, 1, 2): (Eyebrows.EMPTY, Necks.NOD, Arms.EMPTY),
    (0, 1, 3): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (0, 2, 1): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_UP),
    (0, 2, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (0, 2, 3): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_MOUTH),
    (0, 2, 4): (Eyebrows.ANGRY, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 1, 1): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_UP),
    (1, 1, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 1, 3): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_MOUTH),
    (1, 1, 4): (Eyebrows.ANGRY, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 2, 1): (Eyebrows.SAD, Necks.TILT, Arms.LEFT_HAND_CHIN),
    (1, 2, 2): (Eyebrows.NORMAL, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 2, 3): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 3, 1): (Eyebrows.NORMAL, Necks.TILT, Arms.LEFT_HAND_CHIN),
    (1, 3, 2): (Eyebrows.SAD, Necks.TILT, Arms.EMPTY),
    (1, 3, 3): (Eyebrows.HAPPY, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 4, 1): (Eyebrows.NORMAL, Necks.TILT, Arms.EMPTY),
    (1, 4, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (1, 4, 3): (Eyebrows.EMPTY, Necks.EMPTY, Arms.RIGHT_HAND_MOUTH),
    (1, 5, 1): (Eyebrows.NORMAL, Necks.NOD, Arms.EMPTY),
    (1, 5, 2): (Eyebrows.SURPRISED, Necks.EMPTY, Arms.LEFT_HAND_CHIN),
    (1, 5, 3): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (2, 1, 1): (Eyebrows.NORMAL, Necks.TILT, Arms.LEFT_HAND_CHIN),
    (2, 1, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (2, 1, 3): (Eyebrows.HAPPY, Necks.EMPTY, Arms.BANZAI),
    (2, 2, 1): (Eyebrows.HAPPY, Necks.NOD, Arms.EMPTY),
    (2, 2, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (2, 2, 3): (Eyebrows.ANGRY, Necks.DISAGREE, Arms.EMPTY),
    (2, 3, 1): (Eyebrows.SAD, Necks.NATURAL_NOD, Arms.EMPTY),
    (2, 3, 2): (Eyebrows.EMPTY, Necks.EMPTY, Arms.LEFT_HAND_CHIN),
    (2, 3, 3): (Eyebrows.EMPTY, Necks.NOD, Arms.EMPTY),
    (2, 4, 1): (Eyebrows.SAD, Necks.NATURAL_NOD, Arms.EMPTY),
    (2, 4, 2): (Eyebrows.EMPTY, Necks.EMPTY, Arms.LEFT_HAND_CHIN),
    (2, 4, 3): (Eyebrows.EMPTY, Necks.TILT, Arms.EMPTY),
    (2, 5, 1): (Eyebrows.SAD, Necks.TILT, Arms.EMPTY),
    (2, 5, 2): (Eyebrows.SURPRISED, Necks.TILT, Arms.EMPTY),
    (2, 5, 3): (Eyebrows.SAD, Necks.NOD, Arms.EMPTY),
    (3, 1, 1): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_MOUTH),
    (3, 1, 2): (Eyebrows.NORMAL, Necks.TILT, Arms.EMPTY),
    (3, 1, 3): (Eyebrows.EMPTY, Necks.NOD, Arms.EMPTY),
    (3, 2, 1): (Eyebrows.SAD, Necks.NATURAL_NOD, Arms.EMPTY),
    (3, 2, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (3, 2, 3): (Eyebrows.NORMAL, Necks.NOD, Arms.EMPTY),
    (3, 3, 1): (Eyebrows.SURPRISED, Necks.EMPTY, Arms.LEFT_HAND_POINT),
    (3, 3, 2): (Eyebrows.EMPTY, Necks.NATURAL_NOD, Arms.EMPTY),
    (3, 3, 3): (Eyebrows.NORMAL, Necks.DISAGREE, Arms.EMPTY),
    (3, 4, 1): (Eyebrows.NORMAL, Necks.TILT, Arms.EMPTY),
    (3, 4, 2): (Eyebrows.EMPTY, Necks.TILT, Arms.LEFT_HAND_CHIN),
    (3, 5, 1): (Eyebrows.SURPRISED, Necks.EMPTY, Arms.ONEGAI),
    (3, 5, 2): (Eyebrows.EMPTY, Necks.TILT, Arms.LEFT_HAND_CHIN),
    (3, 5, 3): (Eyebrows.HAPPY, Necks.NATURAL_NOD, Arms.EMPTY),
    (4, 1, 1): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_UP),
    (4, 1, 2): (Eyebrows.EMPTY, Necks.NOD, Arms.EMPTY),
    (4, 1, 3): (Eyebrows.EMPTY, Necks.EMPTY, Arms.LEFT_HAND_CHIN),
    (5, 1, 1): (Eyebrows.HAPPY, Necks.NOD, Arms.EMPTY),
    (5, 2, 1): (Eyebrows.HAPPY, Necks.EMPTY, Arms.RIGHT_HAND_BYE),
    (6, 1, 1): (Eyebrows.HAPPY, Necks.NOD, Arms.EMPTY),
    (6, 1, 2): (Eyebrows.EMPTY, Necks.EMPTY, Arms.RIGHT_HAND_BYE), 
}

# 動作を実行する関数
def perform_action(eyebrow_status_message, neck_motion_message, arm_motion_message, sound_path):
    # 首と眉の動きのパブリッシュ
    neck_motion_pub.publish(neck_motion_message)
    eyebrow_status_pub.publish(eyebrow_status_message)
    arm_motion_pub.publish(arm_motion_message)
    rospy.loginfo("Published '{}' to /neck_motion".format(neck_motion_message))
    rospy.loginfo("Published '{}' to /eyebrow_status".format(eyebrow_status_message))
    rospy.loginfo("Published '{}' to /arm_motion".format(arm_motion_message))

    # サウンド再生
    try:
        play_sound(sound_path, topic_name='robotsound_jp', wait=True)
        rospy.loginfo("Playing sound: {}".format(sound_path))
    except Exception as e:
        rospy.logerr("Failed to play sound: {}".format(e))

# 条件に基づいて動作を選択する関数
def action_callback(msg):
    global story, section

    trigger = msg.data  # UInt16の値を取得
    rospy.loginfo("Received trigger: {}".format(trigger))

    action = (story, section, trigger)

    sound_data = "package://kxr_controller/resources/pooh_voice/wav/output_{}_{}_{}.wav".format(*action)

    if trigger == 6:
        perform_action("happy", "", "left_hand_chin", 'package://kxr_controller/resources/pooh_voice/wav/output_thanks.wav')
    elif trigger == 7:
        perform_action("happy", "", "right_hand_bye", 'package://kxr_controller/resources/pooh_voice/wav/output_bye.wav')
    elif trigger == 8:
        perform_action("happy", "", "left_hand_bye", 'package://kxr_controller/resources/pooh_voice/wav/output_bye.wav')
    elif trigger == 9:
        perform_action("happy", "", "right_hand_up", 'package://kxr_controller/resources/pooh_voice/wav/output_hello.wav')
    elif trigger == 10:
        perform_action("happy", "", "left_hand_up", 'package://kxr_controller/resources/pooh_voice/wav/output_hello.wav')       
    elif trigger == 11:
        perform_action("normal", "", "left_hand_chin", 'package://kxr_controller/resources/pooh_voice/wav/output_wait.wav')
    elif trigger == 12:
        perform_action("sad", "nod", "", 'package://kxr_controller/resources/pooh_voice/wav/output_hurt.wav')
    elif trigger == 13:
        perform_action("surprised", "", "left_hand_chin", 'package://kxr_controller/resources/pooh_voice/wav/output_wrong.wav')
    elif trigger == 14:
        perform_action("normal", "tilt", "", 'package://kxr_controller/resources/pooh_voice/wav/output_sleepy.wav')
    elif trigger == 15:
        perform_action("happy", "", "right_hand_up", 'package://kxr_controller/resources/pooh_voice/wav/output_ok.wav')
    elif trigger == 16:
        perform_action("happy", "nod", "", 'package://kxr_controller/resources/pooh_voice/wav/output_nod.wav')
    elif trigger == 17:
        perform_action("normal", "nod", "", 'package://kxr_controller/resources/pooh_voice/wav/output_nod2.wav')
    elif trigger == 18:
        perform_action("normal", "nod", "", 'package://kxr_controller/resources/pooh_voice/wav/output_nod3.wav')
    elif trigger == 19:
        perform_action("normal", "disagree", "", 'package://kxr_controller/resources/pooh_voice/wav/output_disagree.wav')   
    elif trigger == 20:
        perform_action("sad", "tilt", "", 'package://kxr_controller/resources/pooh_voice/wav/output_dontknow.wav')
    elif trigger == 21:
        perform_action("happy", "tilt", "left_hand_chin", 'package://kxr_controller/resources/pooh_voice/wav/output_honey.wav')  
    elif trigger == 22:
        ret = random.randint(0,1)
        if ret == 0:
            perform_action("happy", "", "janken", 'package://kxr_controller/resources/pooh_voice/wav/output_janken_rock.wav')
        elif ret == 1:
            perform_action("happy", "", "janken", 'package://kxr_controller/resources/pooh_voice/wav/output_janken_paper.wav')
    elif trigger == 23:                                                                                                                                                                                   
        ret = random.randint(0,1)                                                                                                                                                                        
        if ret == 0:                                                                                                                                                                                      
            perform_action("happy", "", "aiko", 'package://kxr_controller/resources/pooh_voice/wav/output_aiko_rock.wav')                     
        elif ret == 1:                                                                                                                                                                                      
            perform_action("happy", "", "aiko", 'package://kxr_controller/resources/pooh_voice/wav/output_aiko_paper.wav')      
    elif trigger == 24:                                                                                                                                                                                   
        perform_action("sad", "", "left_hand_chin", 'package://kxr_controller/resources/pooh_voice/wav/output_lose.wav')
    elif trigger == 25:
        perform_action("happy", "", "banzai", 'package://kxr_controller/resources/pooh_voice/wav/output_yeah.wav')
    elif trigger == 26:                                                       
        perform_action("normal", "", "init_pose", 'package://kxr_controller/resources/pooh_voice/wav/silent_3sec.wav') 
    elif trigger == 27:
        perform_action("happy", "", "banzai", 'package://kxr_controller/resources/pooh_voice/wav/silent_3sec.wav')
    elif trigger == 28:                                                                                                                                                                                   
        perform_action("happy", "tilt", "start_shake", 'package://kxr_controller/resources/pooh_voice/wav/output_start_shake.wav')
    elif trigger == 29:                                                                                                                                                                                   
        ret = random.randint(1,3)                                                                                                                                                                        
        if ret == 1:                                                                                                                                                                                      
            perform_action("happy", "tilt", "", 'package://kxr_controller/resources/pooh_voice/wav/output_during_shake1.wav')                                                                            
        elif ret == 2:                                                                                                                                                                                      
            perform_action("happy", "tilt", "", 'package://kxr_controller/resources/pooh_voice/wav/output_during_shake2.wav')
        elif ret == 3:
            perform_action("happy", "tilt", "", 'package://kxr_controller/resources/pooh_voice/wav/output_during_shake3.wav')        
    elif trigger == 30:                                                                                                                                                                                   
        perform_action("happy", "", "init_pose", 'package://kxr_controller/resources/pooh_voice/wav/output_end_shake.wav')   

    elif action in action_motion_pair:
        perform_action(*action_motion_pair.get(action), sound_data)
    else:
        perform_action("", "", "", sound_data)
        rospy.loginfo("No action defined for story={}, section={}, trigger={}.wav, so only play sound".format(story, section, trigger))

# story とsection を設定
def reconfigure_callback(config, level):
    global story, section
    story = config.story
    section = config.section
    rospy.loginfo("Set story:{}, section:{}".format(story, section))
    return config


def main():
    global story, section
    rospy.init_node('action_listener')

    # dynamic reconfigure の設定
    reconfigure_server = Server(PoohScenarioConfig,
                                reconfigure_callback)

    # パブリッシャーの設定
    global neck_motion_pub, eyebrow_status_pub, arm_motion_pub
    neck_motion_pub = rospy.Publisher('/neck_motion', String, queue_size=1)
    eyebrow_status_pub = rospy.Publisher('/eyebrow_status', String, queue_size=1)
    arm_motion_pub = rospy.Publisher('/arm_motion', String, queue_size=1)

    # サブスクライバーの設定
    rospy.Subscriber('/action_trigger', UInt16, action_callback)

    rospy.loginfo("Action listener node started, waiting for messages...")

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
