#!/usr/bin/env python3
import rospy, json, time
from std_msgs.msg import String
from techrie_demo.msg import InteractionEvent

class RobotStatePub:
    def __init__(self):
        rospy.init_node("robot_state_publisher")
        self.emotion = rospy.get_param("/profile/emotion", "neutral")
        self.interest = float(rospy.get_param("/profile/interest", 0.5))
        self.fatigue = float(rospy.get_param("/profile/fatigue", 0.3))
        self.sociality = float(rospy.get_param("/profile/sociality", 0.5))
        self.last_action = ""
        self.mode = "solo"
        self.last_evt_ts = 0.0

        self.pub = rospy.Publisher("/robot/state", String, queue_size=1, latch=True)
        rospy.Subscriber("/emotion/set", String, self._cb_emote, queue_size=10)
        rospy.Subscriber("/interaction_events", InteractionEvent, self._cb_evt, queue_size=50)

        self.rate = rospy.Rate(2)  # 2Hz くらい
        while not rospy.is_shutdown():
            self._publish_snapshot()
            self.rate.sleep()

    def _cb_emote(self, s: String):
        self.emotion = (s.data or "neutral").strip()

    def _cb_evt(self, e: InteractionEvent):
        self.last_action = (e.event_type or "").strip()
        self.last_evt_ts = e.stamp.to_sec()
        # 適当な基準で mode を切り替え（必要ならここを調整）
        self.mode = "with_people" if "HUMAN" in self.last_action or "GPT_ACT" in self.last_action else "solo"

    def _publish_snapshot(self):
        state = {
            "emotion": self.emotion,
            "interest": self.interest,
            "fatigue": self.fatigue,
            "sociality_score": self.sociality,
            "last_action": self.last_action,
            "mode": self.mode,
            "last_event_ts": self.last_evt_ts
        }
        self.pub.publish(String(json.dumps(state, ensure_ascii=False)))

if __name__ == "__main__":
    RobotStatePub()
