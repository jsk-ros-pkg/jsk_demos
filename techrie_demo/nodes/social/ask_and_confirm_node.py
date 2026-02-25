#!/usr/bin/env python3
# nodes/social/ask_and_confirm_node.py
import rospy
from std_msgs.msg import String as S
from sensor_msgs.msg import Joy
from techrie_demo.srv import AskForItem, AskForItemResponse

class AskAndConfirm:
    def __init__(self):
        rospy.init_node('ask_and_confirm_node')

        # ===== Params =====
        # 入力モード: 'human'（/human/ok,/human/no） or 'joy'（/joy のボタン）
        self.input_mode   = rospy.get_param("~input_mode", "human")
        self.ok_topic     = rospy.get_param("~ok_topic", "/human/ok")
        self.ng_topic     = rospy.get_param("~ng_topic", "/human/no")
        self.joy_topic    = rospy.get_param("~joy_topic", "/joy")
        self.OK_BTN       = int(rospy.get_param("~ok_button", 0))
        self.NG_BTN       = int(rospy.get_param("~ng_button", 1))
        self.timeout_sec  = float(rospy.get_param("~timeout_sec", 30.0))

        # ===== State =====
        self._ok_flag = False
        self._ng_flag = False
        self._last_buttons = []
        self._prev_buttons = []

        # ===== Subscribers =====
        if self.input_mode == "human":
            rospy.Subscriber(self.ok_topic, S, self._on_ok, queue_size=10)
            rospy.Subscriber(self.ng_topic, S, self._on_ng, queue_size=10)
            rospy.loginfo("ask_and_confirm: mode=human (%s, %s)", self.ok_topic, self.ng_topic)
        else:
            rospy.Subscriber(self.joy_topic, Joy, self._on_joy, queue_size=20)
            rospy.loginfo("ask_and_confirm: mode=joy (%s, OK=%d NG=%d)",
                          self.joy_topic, self.OK_BTN, self.NG_BTN)

        # ===== Service =====
        rospy.Service('ask_for_item', AskForItem, self._handle)
        rospy.loginfo("ask_for_item ready (mode=%s, timeout=%.1fs)", self.input_mode, self.timeout_sec)

    # ---------- callbacks ----------
    def _on_ok(self, _msg: S): self._ok_flag = True
    def _on_ng(self, _msg: S): self._ng_flag = True

    def _on_joy(self, m: Joy):
        # 立ち上がりエッジを拾う
        btns = list(m.buttons)
        if not self._last_buttons:
            self._prev_buttons = btns[:]  # 初回
        else:
            self._prev_buttons = self._last_buttons[:]
        self._last_buttons = btns

        def pressed(idx):  # 立ち上がり or 押下中
            if idx < 0 or idx >= len(btns): return False
            now = bool(btns[idx])
            prev = bool(self._prev_buttons[idx]) if idx < len(self._prev_buttons) else False
            return now or (now and not prev)

        if pressed(self.OK_BTN): self._ok_flag = True
        if pressed(self.NG_BTN): self._ng_flag = True

    # ---------- service ----------
    def _handle(self, req):
        # リクエスト毎にフラグをクリア
        self._ok_flag = False
        self._ng_flag = False
        start = rospy.Time.now()
        end   = start + rospy.Duration(self.timeout_sec)

        rospy.loginfo("AskForItem: %s ? (waiting up to %.1fs)", req.item_name, self.timeout_sec)
        r = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if self._ok_flag:
                return AskForItemResponse(ok=True,  location_hint="near_palette")
            if self._ng_flag:
                return AskForItemResponse(ok=False, location_hint="")
            r.sleep()

        rospy.loginfo("AskForItem: timeout -> NG")
        return AskForItemResponse(ok=False, location_hint="")

def main():
    AskAndConfirm()
    rospy.spin()

if __name__ == '__main__':
    main()
