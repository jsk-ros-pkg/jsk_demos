#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time, json
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from techrie_demo.msg import InteractionEvent

class GreetingConductor:
    def __init__(self):
        rospy.init_node("greeting_conductor")

        # ========== Params ==========
        # /joy 入力
        self.joy_topic         = rospy.get_param("~joy_topic", "/joy")
        self.btn_open          = int(rospy.get_param("~joy_button_open", 40))  # 開会
        self.btn_close         = int(rospy.get_param("~joy_button_close", 41)) # 閉会
        self.require_longpress = bool(rospy.get_param("~require_longpress", False))
        self.longpress_sec     = float(rospy.get_param("~longpress_sec", 0.5))

        # 予備: 手動トピックでのトリガも残したい場合（不要なら false）
        self.enable_topic_trigger = bool(rospy.get_param("~enable_topic_trigger", False))
        self.topic_open  = rospy.get_param("~topic_open",  "/ceremony/open")
        self.topic_close = rospy.get_param("~topic_close", "/ceremony/close")

        # リピート防止
        self.allow_repeat = bool(rospy.get_param("~allow_repeat", False))
        self.debounce_sec = float(rospy.get_param("~debounce_sec", 2.0))

        # 日記メタ
        self.diary_meta = {
            "project": rospy.get_param("~diary_project", "making_the_moon_together"),
            "phase":   rospy.get_param("~diary_phase",   "opening"),
            "goal":    rospy.get_param("~diary_goal",    "workshop greeting"),
        }

        # ========== Publishers ==========
        self.pub_text = rospy.Publisher("/robot_text", String, queue_size=10)
        self.pub_emo  = rospy.Publisher("/emotion/set", String, queue_size=10)
        self.pub_pose = rospy.Publisher("/motion/play", String, queue_size=10)
        self.pub_lock = rospy.Publisher("/ui/speak_lock", String, queue_size=1)
        self.pub_evt  = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)

        # ========== Subscribers ==========
        rospy.Subscriber(self.joy_topic, Joy, self._on_joy, queue_size=10)
        if self.enable_topic_trigger:
            rospy.Subscriber(self.topic_open,  String, self._on_open_topic,  queue_size=1)
            rospy.Subscriber(self.topic_close, String, self._on_close_topic, queue_size=1)

        # ========== Internal ==========
        self._did_open  = False
        self._did_close = False
        self._last_ts   = 0.0

        self._prev_btns = []                  # 前回のボタン配列
        self._press_start = {}                # {btn_idx: press_start_time}

        rospy.loginfo("greeting_conductor: ready (/joy=%s, open=%d, close=%d, longpress=%s %.2fs)",
                      self.joy_topic, self.btn_open, self.btn_close, self.require_longpress, self.longpress_sec)

    # ------------- Helpers -------------
    def _lock(self, who="greet"):   self.pub_lock.publish(String(who))
    def _unlock(self):              self.pub_lock.publish(String(""))

    def _say(self, text, emo=None):
        self.pub_text.publish(String(text))
        if emo: self.pub_emo.publish(String(emo))
        rospy.sleep(0.2)

    def _pose(self, cmd):
        self.pub_pose.publish(String(cmd))
        # 簡易ウエイト（必要に応じて調整）
        if cmd.startswith("seq:"):
            time.sleep(1.0)
        elif cmd.startswith("pose:"):
            time.sleep(0.8)
        else:
            time.sleep(1.2)

    def _emo(self, name):
        self.pub_emo.publish(String(name))
        rospy.sleep(0.1)

    def _log_evt(self, etype, meta=None):
        ev = InteractionEvent()
        ev.stamp = rospy.Time.now()
        ev.event_type = etype
        ev.actor_id = "greeting_conductor_end"
        ev.target_id = "scene"
        full = dict(self.diary_meta)
        if meta: full.update(meta)
        ev.meta_json = json.dumps(full, ensure_ascii=False)
        self.pub_evt.publish(ev)

    def _debounce_ok(self):
        now = time.time()
        if now - self._last_ts < self.debounce_sec:
            return False
        self._last_ts = now
        return True

    # ------------- Flows -------------
    def _run_open(self):
        if (self._did_open and not self.allow_repeat) or not self._debounce_ok():
            return
        self._lock("greet_open")
        try:
            self._log_evt("CEREMONY_OPEN")
            self._say("みんな")
            self._pose("pose:hi")
            rospy.sleep(3.0)
            self._pose("pose:reset")

            rospy.sleep(3.0)

            self._say("一ヶ月間ありがとう")
            self._pose("seq:nod")
            self._emo("calm")

            rospy.sleep(4.0)
            
            self._say("みんなとお絵描き楽しかった")
            self._pose("traj:draw/stroke_arc_v1")   # 一筆モーション

            rospy.sleep(7.0)
            
            self._say("また会おうね", emo="joy")
            self._pose("pose:joy")
            rospy.sleep(4.0)
            self._pose("pose:reset")
            self._did_open = True
        finally:
            self._unlock()

    def _run_close(self):
        if (self._did_close and not self.allow_repeat) or not self._debounce_ok():
            return
        self._lock("greet_close")
        try:
            self._log_evt("CEREMONY_CLOSE")

            self._say("みんな、今日はありがとう")
            self._pose("seq:nod")
            self._emo("calm")

            rospy.sleep(4.0)
            
            self._say("いっしょに出来て嬉しかった", emo="joy")

            rospy.sleep(7.0)
            self._say("来週からもよろしくね")
            self._pose("pose:joy")
            rospy.sleep(4.0)
            self._pose("pose:reset")
            self._did_close = True
        finally:
            self._unlock()

    # ------------- Triggers -------------
    def _on_open_topic(self, _):  self._run_open()
    def _on_close_topic(self, _): self._run_close()

    def _on_joy(self, msg: Joy):
        btns = list(msg.buttons) if msg.buttons else []

        # 配列長が短い場合に備えつつ、対象ボタンの状態を取得
        def _is_down(i): return (i < len(btns) and btns[i] == 1)

        # 初回は prev を埋めるだけ
        if not self._prev_btns:
            self._prev_btns = btns
            return

        # 対象ボタンの処理をまとめる
        for b_idx, handler in ((self.btn_open, self._run_open), (self.btn_close, self._run_close)):
            prev = (b_idx < len(self._prev_btns) and self._prev_btns[b_idx] == 1)
            cur  = _is_down(b_idx)

            # 長押しが必要な場合
            if self.require_longpress:
                if cur and not prev:
                    # 押し始め
                    self._press_start[b_idx] = time.time()
                elif not cur and prev:
                    # 離した瞬間に判定
                    t0 = self._press_start.pop(b_idx, None)
                    if t0 is not None and (time.time() - t0) >= self.longpress_sec:
                        rospy.loginfo("greeting_conductor: long-press btn=%d", b_idx)
                        handler()
            else:
                # 立ち上がりエッジで実行
                if cur and not prev:
                    rospy.loginfo("greeting_conductor: pressed btn=%d", b_idx)
                    handler()

        self._prev_btns = btns

def main():
    GreetingConductor()
    rospy.spin()

if __name__ == "__main__":
    main()
