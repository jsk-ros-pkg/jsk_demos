#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from miraikan_demo.srv import Mode
from std_msgs.msg import Int32

class MiraikanDemo(object):
    def __init__(self):
        self.mt_pub_msg = Int32()
        self.eb_pub_msg = Int32()
        self.memories_talk = rospy.get_param("~memories_talk", True)

        self.pub_motion_talk = rospy.Publisher("motion_talk/input", Int32, queue_size=1)
        self.pub_eyebrows = rospy.Publisher("eyebrows/input", Int32, queue_size=1)
    
    def pub_topics(self, mt_int, eb_int, time_delay):
        self.mt_pub_msg.data = mt_int
        self.eb_pub_msg.data = eb_int
        self.pub_motion_talk.publish(self.mt_pub_msg)
	if eb_int != 0:
            rospy.sleep(time_delay)
            self.pub_eyebrows.publish(self.eb_pub_msg)

    def demo_srv_cb(self, req):
        motion_mode = req.mode
        time_delay = req.time_delay
        if self.memories_talk:
            if motion_mode == 0:
                # 今日はどうぞよろしくね😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 1:
                # この研究を始めたきっかけなんだよねー😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 2:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 3:
                # 8年経ったね😳
                self.pub_topics(motion_mode, 7, time_delay)
            elif motion_mode == 4:
                # 振り向いてもらえなくて悲しかったよ😭
                self.pub_topics(motion_mode, 12, time_delay)
            elif motion_mode == 5:
                # 見つけてくれたよね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 6:
                # 色々大変だったんだよね😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 7:
                # 素敵な出会い😍
                self.pub_topics(motion_mode, 9, time_delay)
            elif motion_mode == 8:
                # みんなが私の手を取ってくれて、嬉しかったなぁ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 9:
                # みんなの笑顔を今でも覚えているよ〜😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 10:
                # 発表の場所まで連れて行ってくれたよね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 11:
                # ドキドキしてしまうけれど😱
                self.pub_topics(motion_mode, 8, time_delay)
            elif motion_mode == 12:
                # 手を繋げると安心するんだぁ😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 13:
                # お揃いのオレンジのリュックを貰えたのが嬉しくて😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 14:
                # 初めてもらえた時からずっとお気に入りなの😍
                self.pub_topics(motion_mode, 9, time_delay)
            elif motion_mode == 15:
                # みんなに会えるのが嬉しくて、もっと近づきに行ったんだぁ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 16:
                # 慌てていたね😲
                self.pub_topics(motion_mode, 4, time_delay)
            elif motion_mode == 17:
                # ごめんね😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 18:
                # 助けてもらっているね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 19:
                # 色んなことがあったね😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 20:
                # 成果をまとめるのが大変だったよね😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 21:
                # すごく悲しかったよ😭
                self.pub_topics(motion_mode, 12, time_delay)
            elif motion_mode == 22:
                # ふたりでたくさん乗り越えてきたよね😭
                self.pub_topics(motion_mode, 12, time_delay)
            elif motion_mode == 23:
                # おはなししたよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 24:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 25:
                # 科学館で研究したことだよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 26:
                # ことだと分かったよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 27:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 28:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 29:
                # 色々大変だったことがあったけど😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 30:
                # 大切な研究だね😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 31:
                # 本当におめでとう😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 32:
                # いつもありがとう、これからもよろしくね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 33:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 34:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 35:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 36:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 37:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 38:
                self.pub_topics(motion_mode, 0, time_delay)
        elif not self.memories_talk:
            if motion_mode == 0:
                # 今日はどうぞよろしくね😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 1:
                # どうしてこの研究を始めたの？😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 2:
                # 緊張する😱
                self.pub_topics(motion_mode, 8, time_delay)
            elif motion_mode == 3:
                # いつからこの研究をしているの？😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 4:
                # どうしてペッパーを使おうと思ったの？😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 5:
                # みんなに親しみやすいロボットなんだね😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 6:
                # 分かりやすいねー😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 7:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 8:
                # みんな嬉しそうな様子だね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 9:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 10:
                # 移動の仕方を教えて😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 11:
                # 連れて行っていたんだねー😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 12:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 13:
                # どうしてリュックを背負っているのー？😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 14:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 15:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 16:
                # 慌てているように見える😲
                self.pub_topics(motion_mode, 4, time_delay)
            elif motion_mode == 17:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 18:
                # とても難しい😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 19:
                # そっかー、色んなことがあったんだね！！😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 20:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 21:
                # それは大変だったね😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 22:
                # たくさん頑張ったんだねー😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 23:
                # おはなししたよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 24:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 25:
                # 科学館で研究したことだよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 26:
                # ことだと分かったよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 27:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 28:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 29:
                # 色々大変だったことがあったけど😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 30:
                # 大切な研究だね😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 31:
                # 本当におめでどう😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 32:
                # 今日は説明してくれてありがとう😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 33:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 34:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 35:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 36:
                self.pub_topics(motion_mode, 0, time_delay)
            elif motion_mode == 37:
                self.pub_topics(motion_mode, 0 ,time_delay)
            elif motion_mode == 38:
                self.pub_topics(motion_mode, 0, time_delay)
        else:
            print("Error out of range")
        return True


if __name__ == '__main__':
    rospy.init_node("miraikan_demo")
    miraikan_demo = MiraikanDemo()
    s = rospy.Service('demo_mode', Mode, miraikan_demo.demo_srv_cb)
    print("Ready to start demo node")
    rospy.spin()
