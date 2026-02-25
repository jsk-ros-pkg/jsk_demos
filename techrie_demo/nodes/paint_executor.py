#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, actionlib, random, json, time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Twist

from techrie_demo.msg import (
    PaintStrokeAction, PaintStrokeResult,
    InteractionEvent,
)
from techrie_demo.srv import AskForItem, AskForItemRequest

# optional: サーボ制御メッセージ（あれば使う）
try:
    from kxr_controller.msg import ServoOnOff
    HAS_SERVO = True
except Exception:
    HAS_SERVO = False


class PaintExecutor:
    def __init__(self):
        rospy.init_node("paint_executor")

        # ===== Params =====
        # 描画ベース
        self.stroke_bank = rospy.get_param("~stroke_bank", [
            "draw/stroke_arc_v1","draw/stroke_arc_v2",
            "draw/stroke_zig_v1","draw/stroke_zig_v2",
        ])
        self.traj_reach_paint = rospy.get_param("~traj_reach_paint", "draw/reach_paint")
        self.strokes_min = int(rospy.get_param("~strokes_min", 3))
        self.strokes_max = int(rospy.get_param("~strokes_max", 6))

        # ヒト確認フロー
        self.ask_service = rospy.get_param("~ask_service_name", "ask_for_item")
        self.ask_timeout = float(rospy.get_param("~ask_timeout_sec", 12.0))
        self.announce    = bool(rospy.get_param("~announce_via_text", True))

        # place_ready リトライ設定
        self.ask_place_max_retries = int(rospy.get_param("~ask_place_max_retries", 2))   # タイムアウト後の追加リトライ回数
        self.ask_place_reprompt    = bool(rospy.get_param("~ask_place_reprompt", True))  # 毎回プロンプトを出し直す

        # 同期系
        self.use_motion_done = bool(rospy.get_param("~use_motion_done", True))
        self.motion_done_topic = rospy.get_param("~motion_done_topic", "/motion_done")

        # サーボ制御（任意）
        self.use_servo_topic = bool(rospy.get_param("~use_servo_topic", False))
        self.servo_topic = rospy.get_param("~servo_topic", "/servo_on_off")
        self.servo_names_on = rospy.get_param("~servo_names_on", ["rarm"])

        # 色の日本語対応
        self.color_aliases = rospy.get_param("~color_aliases", {
            "red":"赤","blue":"青","yellow":"黄","green":"緑",
            "orange":"オレンジ","purple":"紫","pink":"ピンク",
            "white":"白","black":"黒","brown":"茶","cyan":"シアン","magenta":"マゼンタ",
        })
        self.color_palette = rospy.get_param("~color_palette",
            ['red','blue','yellow','green','orange','purple','pink'])
        self.max_color_queries = int(rospy.get_param("~max_color_queries", 2))

        # 割り込みポリシー
        self.allow_interrupts     = bool(rospy.get_param("~allow_interrupts", True))
        self.interrupt_after_sec  = float(rospy.get_param("~interrupt_after_sec", 1.0))  # 開始直後は割り込まない
        self.interrupt_prob       = float(rospy.get_param("~interrupt_prob", 0.7))       # 受ける確率（ベース）
        self.soft_interrupt       = bool(rospy.get_param("~soft_interrupt", True))       # True=ストロークの切れ目で割り込む
        self.interrupt_priority = rospy.get_param("~interrupt_priority",
            ["praise","pet","show_art","offer_food","invite"])
        self.interrupt_policy_map = rospy.get_param("~interrupt_policy_map", {
            "praise": "praise",
            "pet": "pet",
            "show_art": "observe_human_show",
            "offer_food": "eat",
            "invite": "invite",
        })
        self.interrupt_prob_by_event = rospy.get_param("~interrupt_prob_by_event", {
            # "praise": 0.9, "pet": 0.8, ...
        })

        # ===== IO =====
        self.pub_pose   = rospy.Publisher("/motion/play", String, queue_size=10)  # すべて /motion/play に流す
        self.pub_event  = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)
        self.pub_text   = rospy.Publisher("/robot_text", String, queue_size=10)
        self.pub_emote  = rospy.Publisher("/emotion/set", String, queue_size=10)
        self.pub_color  = rospy.Publisher("/paint_color", String, queue_size=1, latch=True)
        self.lock_pub   = rospy.Publisher("/ui/speak_lock", String, queue_size=1)
        self.policy_pub = rospy.Publisher("/policy/next_action", String, queue_size=1)
        self.look_pub   = rospy.Publisher("/detected_object_pose", String, queue_size=1)
        
        # ★ 追加：idle停止用フラグ
        self.idle_pause_pub = rospy.Publisher("/idle/paused", Bool, queue_size=1, latch=True)

        # ベース移動用
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/ridgeback_control/cmd_vel")
        self.pre_move_prob = float(rospy.get_param("~pre_move_prob", 0.4)) # たまに動く確率
        self.pre_move_duration = float(rospy.get_param("~pre_move_duration", 0.1)) # [s]
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        # 色のラッチ（外部 color_policy から上書きされうる）
        # 初期色は固定せずランダムに
        try:
            palette = self.color_palette  # 既に持っている場合
        except AttributeError:
            palette = rospy.get_param('~palette', ['red','blue','yellow','green','orange','purple','pink'])
        self.last_color = random.choice(palette)

        rospy.Subscriber("/paint_color", String, self._color_cb, queue_size=1)

        # サーボ出力（任意）
        self.servo_pub = None
        if self.use_servo_topic and HAS_SERVO:
            self.servo_pub = rospy.Publisher(self.servo_topic, ServoOnOff, queue_size=10, latch=True)

        # Ask サービス
        self.ask = None
        rospy.wait_for_service(self.ask_service)
        self.ask = rospy.ServiceProxy(self.ask_service, AskForItem)

        # human event -> recent map
        self._recent = {}
        def _mkcb(name):
            def _cb(_msg):
                self._mark_recent(name)
            return _cb
        rospy.Subscriber("/human/praise",     String, _mkcb("praise"),     queue_size=10)
        rospy.Subscriber("/human/pet",        String, _mkcb("pet"),        queue_size=10)
        rospy.Subscriber("/human/show_art",   String, _mkcb("show_art"),   queue_size=10)
        rospy.Subscriber("/human/offer_food", String, _mkcb("offer_food"), queue_size=10)
        rospy.Subscriber("/human/invite",     String, _mkcb("invite"),     queue_size=10)
        rospy.Subscriber("/paint_color_policy", String, self._color_cb, queue_size=1)

        # アクションサーバ
        self.server = actionlib.SimpleActionServer(
            "paint_stroke", PaintStrokeAction, execute_cb=self.execute, auto_start=False
        )
        self.server.start()
        rospy.loginfo("paint_executor ready")

        # オプション：描いたら SHOW に誘導
        self.hint_show_after = bool(rospy.get_param("~hint_show_after_paint", False))

    # ---------- small helpers ---------
    def _lock(self):   self.lock_pub.publish(String("paint_executor"))
    def _unlock(self): self.lock_pub.publish(String(""))

    def _say(self, text, color="white"):
        if not self.announce: return
        self.pub_text.publish(String(text))
        self.pub_emote.publish(String(color))
        rospy.loginfo("[paint_executor] %s", text)

    def _canon_color(self, c):
        c = (c or "").strip().lower()
        return c

    def _canon_color_jp(self, c):
        key = self._canon_color(c)
        return self.color_aliases.get(key, key or "（色）")

    def _color_cb(self, msg: String):
        c = self._canon_color(msg.data)
        if c:
            self.last_color = c

    def _play_pose(self, name):
        self.pub_pose.publish(String(name))

    def _wait_motion_done(self, timeout=10.0) -> bool:
        if not self.use_motion_done:
            rospy.sleep(0.05)
            return True
        try:
            msg = rospy.wait_for_message(self.motion_done_topic, Bool, timeout=timeout)
            return bool(msg.data)
        except Exception as e:
            rospy.logwarn("wait_motion_done timeout or error: %s", e)
            return False

    # ---------- ask helpers ----------
    def _ask_ok(self, item) -> bool:
        try:
            resp = self.ask(AskForItemRequest(item_name=item))
            rospy.loginfo("ask(%s) -> %s", item, resp.ok)
            return bool(resp.ok)
        except Exception as e:
            rospy.logwarn("ask failed: %s", e)
            return False

    def _ask_until_ok(self, item, first_prompt=None, retry_prompt=None, retries=2):
        """
        ask_for_item を最大 retries 回までリトライ。
        各リトライは ask_timeout 秒の“待ち窓”で 0 を監視する。
        0 が押されなければ「まだみたい…」と言って次のリトライへ。
        """
        attempt = 0
        while not rospy.is_shutdown() and attempt <= retries:
            if attempt == 0 and first_prompt:
                self._say(first_prompt, "cyan")
            elif attempt > 0 and retry_prompt:
                self._say(retry_prompt, "cyan")

            deadline = time.time() + float(self.ask_timeout)
            while not rospy.is_shutdown() and time.time() < deadline:
                if self._ask_ok(item):
                    return True
                rospy.sleep(0.2)

            attempt += 1
            if attempt <= retries:
                self._say("まだみたい…", "sad")

        return False

    # ---------- servo helpers ----------
    def _servo_on(self):
        if self.servo_pub:
            msg = ServoOnOff()
            msg.joint_names = list(self.servo_names_on)
            msg.servo_on_states = [True] * len(msg.joint_names)
            self.servo_pub.publish(msg)

    def _servo_all_off(self):
        if self.servo_pub:
            msg = ServoOnOff()
            msg.joint_names = [""]      # robot_behavior 側で全OFFを受ける実装
            msg.servo_on_states = [False]
            self.servo_pub.publish(msg)

    def _servo_all_on(self):
        if self.servo_pub:
            msg = ServoOnOff()
            msg.joint_names = [""]      # robot_behavior 側で全ONを受ける実装
            msg.servo_on_states = [True]
            self.servo_pub.publish(msg)

    # ---------- base move helpers ----------
    def _publish_cmd_vel(self, y_speed: float, duration: float):
        """
        側方移動の速度コマンドを一定時間出す。前後(x)や回転は0固定。
        y_speed は ±0.1 [m/s] を想定。duration は [s]。
        """
        if not hasattr(self, "cmd_vel_pub") or self.cmd_vel_pub is None:
            return
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = float(y_speed)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0


        rate = rospy.Rate(10)
        t_end = time.time() + float(duration)
        while not rospy.is_shutdown() and time.time() < t_end:
            self.cmd_vel_pub.publish(msg)
            rate.sleep()
        # 停止
        self.cmd_vel_pub.publish(Twist())


    def _maybe_side_step(self):
        """
        描き始める直前に、確率 pre_move_prob で左右どちらかへ側方移動する。
        要件：前後しない。送る距離(速度の絶対値)は 0.1。
        """
        try:
            if random.random() > self.pre_move_prob:
                return
            # 右/左を 50% で選択。右 = +y、左 = -y
            direction = random.choice(["right", "left"])
            y_speed = -0.1 if direction == "right" else 0.1 # ★ 速度の絶対値を常に 0.1
            # アナウンス
            self._say("右に行くよ〜" if direction == "right" else "左に行くよ〜", "joy")
            # 送出
            self._publish_cmd_vel(y_speed, self.pre_move_duration)
        except Exception as e:
            rospy.logwarn("side_step failed: %s", e)

    # ---------- motion helpers ----------
    def _play_traj_and_wait(self, base_name, done_timeout=10.0) -> bool:
        self._play_pose("traj:" + base_name)
        return self._wait_motion_done(done_timeout)

    # ---------- interrupt helpers ----------
    def _mark_recent(self, name):
        self._recent[name] = time.time()

    def _pick_recent_event(self, window_sec=5.0):
        if not self._recent:
            return None
        now = time.time()
        for name in self.interrupt_priority:
            ts = self._recent.get(name, 0.0)
            if now - ts <= window_sec:
                return name
        return None

    def _maybe_interrupt(self, started_at) -> bool:
        if not self.allow_interrupts:
            return False
        if (time.time() - started_at) < self.interrupt_after_sec:
            return False

        evt = self._pick_recent_event(window_sec=5.0)
        if not evt:
            return False

        p = self.interrupt_prob_by_event.get(evt, self.interrupt_prob)
        if random.random() > p:
            return False

        next_state = self.interrupt_policy_map.get(evt)
        if not next_state:
            return False

        self._say("ちょっと待ってね…", "calm")
        self.policy_pub.publish(String(next_state))
        self.server.set_succeeded(PaintStrokeResult())
        return True

    # ---------- main ----------
    def execute(self, goal):
        self._lock()
        started_at = time.time()
        paused_idle = False
        try:
            # 初期化
            self._play_pose("pose:reset")
            rospy.sleep(5.0)

            self._play_pose("pose:hi")
            rospy.sleep(5.0)

            self._play_pose("pose:reset")
            rospy.sleep(5.0)

            # 0) 色の確認
            chosen = self._canon_color(getattr(goal, "color", "")) or self.last_color
            jp = self._canon_color_jp(chosen)
            self._say(f"{jp} で描くね！OK?", "interest")
            rospy.sleep(4.0)

            # 1) 準備：全サーボOFF → OK(=0)待ち（タイムアウトで「まだみたい…」→リトライ）
            self._servo_all_off()
            first_prompt = "絵の具をつけたらOKを押して"
            retry_prompt = "もう一度待つね" if self.ask_place_reprompt else None
            if not self._ask_until_ok("place_ready", first_prompt, retry_prompt, retries=self.ask_place_max_retries):
                self._say("まだみたい…", "sad")
                self._servo_all_on()  # 安全のため再開
                self.server.set_aborted(PaintStrokeResult(), "place not ready")
                return

            # 2) 準備OK：全サーボONに戻して描画へ
            self._say("OK！始めるよ", "happy")
            self._servo_all_on()

            # ★ ここから idle を停止（look が奪われないように）
            self.idle_pause_pub.publish(Bool(True))
            paused_idle = True

            rospy.sleep(5.0)
            # 顔をキャンバス側へ向けるなど（任意）
            self.look_pub.publish(String("center_bottom"))

            # 描き始める前に、たまに左右どちらかへ側方移動（±0.1 m/s）
            self._maybe_side_step()

            # 3) 描く（ストロークごとに割り込み判定）
            n = random.randint(self.strokes_min, self.strokes_max)
            self._say("描くよ〜", "joy")
            for _ in range(n):
                if self.soft_interrupt and self._maybe_interrupt(started_at):
                    return

                base = random.choice(self.stroke_bank)
                if not self._play_traj_and_wait(base, done_timeout=10.0):
                    self.server.set_aborted(PaintStrokeResult(), "stroke timeout")
                    return

                if self._maybe_interrupt(started_at):
                    return

                if self.server.is_preempt_requested():
                    self._say("途中で中断するね", "calm")
                    self.server.set_preempted(PaintStrokeResult())
                    return

            # 4) 終了：見せる→正面へ→idle再開
            if self.hint_show_after:
                self.look_pub.publish(String("center_center"))
                self._say("描けたよ！見てみて〜", "show")
                self.policy_pub.publish(String("show"))
                rospy.sleep(3.0)  # 見せる間
                

            # ★ idle を再開
            if paused_idle:
                self.idle_pause_pub.publish(Bool(False))
                paused_idle = False

            self.server.set_succeeded(PaintStrokeResult())

        finally:
            # 例外・中断でも idle を確実に再開
            if paused_idle:
                self.look_pub.publish(String("center_center"))
                self.idle_pause_pub.publish(Bool(False))
            self._unlock()


def main():
    PaintExecutor()
    rospy.spin()

if __name__ == "__main__":
    main()
