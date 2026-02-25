#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, yaml, time, json
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from techrie_demo.msg import InteractionEvent

SUPPORTED = {"praise","pet","offer_food","show_art","invite","joke","sorry",
             "greeting_hello","greeting_goodnight","ok","no", "dance","dance_stop","dance_fast","dance_slow","dance_mode_clap","dance_mode_random","sing"}

class HumanInputBridge:
    def __init__(self):
        rospy.init_node("human_input_bridge")

        # 設定
        self.cooldown = float(rospy.get_param("~cooldown_sec", 0.3))
        self.map_file = rospy.get_param("~map_file", rospy.get_param("~default_map",
                           rospy.get_param("~pkg_root", "")))  # 互換
        if not self.map_file:
            # デフォルト: パッケージ直下 config/button_map.yaml
            from rospkg import RosPack
            self.map_file = RosPack().get_path("techrie_demo") + "/config/button_map.yaml"

        with open(self.map_file, "r", encoding="utf-8") as f:
            self.button_map = yaml.safe_load(f) or {}

        # 出力
        self.pub_human = {k: rospy.Publisher(f"/human/{k}", String, queue_size=10)
                          for k in SUPPORTED}
        self.pub_event = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=50)

        # 内部状態
        self.prev = []
        self.last_fire = {}  # btn_index -> timestamp

        # 入力購読
        rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=50)
        rospy.loginfo("human_input_bridge: map=%s cooldown=%.2fs", self.map_file, self.cooldown)

    def cb_joy(self, msg: Joy):
        # 初回 prev セット
        if not self.prev:
            self.prev = [0]*len(msg.buttons)

        n = min(len(msg.buttons), len(self.prev))
        now = time.time()

        for i in range(n):
            cur = int(msg.buttons[i])
            prv = int(self.prev[i])
            # 立ち上がりのみ
            if cur == 1 and prv == 0:
                # マップに無い/ignore はスキップ
                intent = self.button_map.get(i, "ignore")
                if intent == "ignore":
                    continue
                # 連打ガード
                if now - self.last_fire.get(i, 0.0) < self.cooldown:
                    continue
                self.last_fire[i] = now

                # /human/<intent> へ publish（未知 intent は無視）
                if intent in SUPPORTED:
                    self.pub_human[intent].publish(String(""))
                    # /interaction_events へも記録（HUMAN_*）
                    ev = InteractionEvent()
                    ev.stamp = rospy.Time.now()
                    ev.event_type = f"HUMAN_{intent.upper()}"
                    ev.actor_id = "human"
                    ev.target_id = "robot"
                    ev.intensity = 1.0
                    ev.meta_json = json.dumps({"button": i})
                    self.pub_event.publish(ev)
                    rospy.loginfo("human_input_bridge: btn %d -> /human/%s", i, intent)
                else:
                    rospy.logwarn("human_input_bridge: btn %d mapped to unsupported intent '%s'", i, intent)

        self.prev = list(msg.buttons)

def main():
    HumanInputBridge()
    rospy.spin()

if __name__ == "__main__":
    main()

