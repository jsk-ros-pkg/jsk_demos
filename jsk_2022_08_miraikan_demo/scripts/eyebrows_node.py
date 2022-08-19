#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

import os
import requests
import json

class TopicToEyebrows(object):
    def __init__(self):
        self.server_ip = rospy.get_param("~server_ip", "")
        rospy.Subscriber("~input", Int32, self.eyebrows_cb)

    def post_mode(self, input_mode):
        """
        0. Normal
        1. Happy（嬉しい・ごきげん）😀
        2. Relieved（安心）😌
        3. Smirking（悪だくみ）😏
        4. Astonished（驚き）😲
        5. Unpleasant（不愉快・冷や汗）😓
        6. Angry（怒り）😠
        7. Flushed（照れ）😳
        8. Fearful（恐怖）😱
        9. Love（好き）😍
        10. Squinting（おふざけ・遊び心）😉
        11. Boring（退屈）😪
        12. Cry（泣き）😭
        """
        degrees = [0, 20, 90, 120, 50, 30, 20, 60, 70, 120, 70, 130, 50]
        wait_times = [5] * 13
        # wait_times[5] = 8.0
        headers = {
            'Accept': 'application/json',
        }
        data = {
            'mode': input_mode,
            'degree': degrees[input_mode],
        }
        try:
            response = requests.post('http://' + self.server_ip + ':3000/api/info', headers=headers, data=data, timeout=(3.0, wait_times[input_mode]))
        except:
            pass  # timeoutを設定しないと2回目以降のpublishができない
    
    def eyebrows_cb(self, msg):
        self.post_mode(msg.data)
        self.post_mode(0)  # 表情を元に戻す

if __name__ == '__main__':
    rospy.init_node("topic_to_eyebrows")
    topic_to_eyebrows = TopicToEyebrows()
    rospy.spin()
