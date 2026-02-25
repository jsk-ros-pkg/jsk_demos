#!/usr/bin/env python3
# sing_controller.py
import rospy, json
from std_msgs.msg import String

import os, sys, rospkg
pkg = rospkg.RosPack().get_path('techrie_demo')
sys.path.insert(0, os.path.join(pkg, 'nodes', 'express'))

# make_tone.py にあなたが追加した歌メロ生成関数を利用
import make_tone

class SingController:
    def __init__(self):
        self.pub = rospy.Publisher('tone_sequence', String, queue_size=10)
        rospy.Subscriber('/human/sing', String, self.on_sing, queue_size=10)
        self.bpm = rospy.get_param('~bpm', 96)

    def on_sing(self, _msg):
        try:
            if hasattr(make_tone, 'generate_twinkle'):
                tones = make_tone.generate_twinkle(bpm=self.bpm)
            elif hasattr(make_tone, 'generate_sing_sequence'):
                # フォールバック: きらきら星（前半）
                notes = ["C4","C4","G4","G4","A4","A4","G4",
                         "F4","F4","E4","E4","D4","D4","C4"]
                durs  = [1,1,1,1,1,1,2, 1,1,1,1,1,1,2]
                tones = make_tone.generate_sing_sequence(notes, durs, bpm=self.bpm)
            else:
                tones = [{"freq": 440, "duration": 250}] * 8
        except Exception as e:
            rospy.logwarn("sing_controller: fallback due to %s", e)
            tones = [{"freq": 440, "duration": 250}] * 8

        self.pub.publish(json.dumps(tones, ensure_ascii=False))
        rospy.loginfo("sing_controller: published %d notes", len(tones))

if __name__ == '__main__':
    rospy.init_node('sing_controller')
    SingController()
    rospy.loginfo("sing_controller ready: waiting /human/sing")
    rospy.spin()
