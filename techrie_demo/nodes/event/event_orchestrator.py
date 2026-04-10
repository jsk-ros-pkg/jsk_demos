#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, actionlib, json, time, random, math
from std_msgs.msg import String, Bool

# 既存メッセージ
from techrie_demo.msg import (
    PaintStrokeAction, PaintStrokeGoal, PaintStrokeResult,
    ShowArtAction,   ShowArtGoal,
    InteractionEvent,
)

# 0/1 ボタンの簡易確認に使う（無ければフォールバック）
try:
    from techrie_demo.srv import AskForItem, AskForItemRequest
    HAS_ASK = True
except Exception:
    HAS_ASK = False


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, float(x)))

def sigmoid(x):
    try:
        return 1.0/(1.0+math.exp(-x))
    except Exception:
        return 0.5 if x==0 else (1.0 if x>0 else 0.0)


class EventOrchestrator:
    def __init__(self):
        rospy.init_node("event_orchestrator")

        # ===== Params =====
        # 入出力トピック
        self.invite_topic        = rospy.get_param("~invite_topic",        "/human/invite")
        self.human_show_topic    = rospy.get_param("~human_show_topic",    "/human/show_art")
        self.human_praise_topic  = rospy.get_param("~human_praise_topic",  "/human/praise")
        self.human_pet_topic     = rospy.get_param("~human_pet_topic",     "/human/pet")
        self.robot_show_topic    = rospy.get_param("~robot_show_topic",    "/robot/show")
        self.robot_praise_topic  = rospy.get_param("~robot_praise_topic",  "/robot/praise")
        self.speak_lock_ns       = rospy.get_param("~speak_lock_ns",       "/ui/speak_lock")

        # 表示・待ち時間
        self.announce            = bool(rospy.get_param("~announce_via_text", True))
        self.paint_result_wait   = float(rospy.get_param("~paint_result_wait_sec", 120.0))
        self.auto_show_after_paint        = bool(rospy.get_param("~auto_show_after_paint", True))
        self.auto_robot_praise_after_show = bool(rospy.get_param("~auto_robot_praise_after_show", False))

        # いじけ（SULK）: /interaction_events の SULK_START を受ける
        self.default_sulk_dur    = float(rospy.get_param("~default_sulk_dur", 12.0))

        # === ★ ドライブ（欲求2軸） ===
        self.drive_min        = float(rospy.get_param("~drive_min", 0.0))
        self.drive_max        = float(rospy.get_param("~drive_max", 1.0))

        self.create_init      = float(rospy.get_param("~create_init", 0.55))  # お絵描き欲 初期値
        self.connect_init     = float(rospy.get_param("~connect_init", 0.50)) # 関わり欲   初期値
        self.create_base      = float(rospy.get_param("~create_base", 0.50))  # 経時回帰の基準
        self.connect_base     = float(rospy.get_param("~connect_base", 0.50))

        # 1秒あたりの回復/減衰量（ベースへ戻る速度）
        self.create_relax_per_sec  = float(rospy.get_param("~create_relax_per_sec", 0.015))
        self.connect_relax_per_sec = float(rospy.get_param("~connect_relax_per_sec", 0.015))

        # イベントでの増分
        self.bump_on_praise    = float(rospy.get_param("~bump_on_praise",   0.10))  # 褒め→関わり↑
        self.bump_on_pet       = float(rospy.get_param("~bump_on_pet",      0.08))  # なで→関わり↑
        self.bump_on_invite    = float(rospy.get_param("~bump_on_invite",   0.06))  # 誘い→関わり↑
        self.bump_on_paint_end = float(rospy.get_param("~bump_on_paint_end",0.10))  # 描けた→自己充実↑
        self.bump_on_nohuman   = float(rospy.get_param("~bump_on_nohuman",  0.06))  # 反応なし→自己充実↑（自発要因）

        # 自発で描き始める条件
        self.enable_self_paint    = bool(rospy.get_param("~enable_self_paint", True))
        self.self_paint_threshold = float(rospy.get_param("~self_paint_threshold", 0.62))
        self.self_margin_needed   = float(rospy.get_param("~self_margin_needed", 0.06))  # create - connect
        self.self_cooldown_sec    = float(rospy.get_param("~self_cooldown_sec", 25.0))
        self.self_need_confirm    = bool(rospy.get_param("~selfdrive_need_confirm", True))  # 0/1で確認するか
        self.self_confirm_wait    = float(rospy.get_param("~selfdrive_gate_wait_sec", 12.0))
        self.self_invite_phrase   = rospy.get_param("~selfdrive_phrase", "ちょっと描いてみてもいい？ 0=OK / 1=あとで")

        # 割り込み確率をドライブ差から計算する（connect が高いほど中断しやすい）
        self.use_drive_interrupt  = bool(rospy.get_param("~use_drive_interrupt", True))
        self.yield_k              = float(rospy.get_param("~yield_k", 6.0))
        self.yield_b              = float(rospy.get_param("~yield_b", 0.0))

        # ドライブ計算の周期
        self.tick_hz              = float(rospy.get_param("~tick_hz", 2.0))
        self.nohuman_push_after   = float(rospy.get_param("~nohuman_push_after_sec", 20.0))

        # ★ 旧 selfdrive パラメータ（互換維持）
        self.selfdrive_enabled            = bool(rospy.get_param("~selfdrive_enabled", True))
        self.selfdrive_check_period_sec   = float(rospy.get_param("~selfdrive_check_period_sec", 5.0))
        self.selfdrive_min_idle_sec       = float(rospy.get_param("~selfdrive_min_idle_sec", 60.0))
        self.selfdrive_interval_sec       = float(rospy.get_param("~selfdrive_interval_sec", 90.0))

        # ★ 割り込みポリシー（queue/cancel）
        self.interrupt_enabled   = bool(rospy.get_param("~interrupt_enabled", True))
        self.interrupt_policy    = rospy.get_param("~interrupt_policy", "queue")  # 'queue' or 'cancel'
        self.interrupt_prob      = float(rospy.get_param("~interrupt_prob", 0.4)) # フォールバック用確率
        self.human_priority_window_sec = float(rospy.get_param("~human_priority_window_sec", 1.5))

        # 日記メタ（イベント文脈）
        self.diary_meta = {
            "project": rospy.get_param("~diary_project", "making_the_moon_together"),
            "phase":   rospy.get_param("~diary_phase",   "workshop_general"),
            "goal":    rospy.get_param("~diary_goal",    "co-create art pieces for the Moon"),
        }

        # ===== IO =====
        self.pub_text  = rospy.Publisher("/robot_text",  String, queue_size=10)
        self.pub_emo   = rospy.Publisher("/emotion/set", String, queue_size=10)
        self.pub_pose  = rospy.Publisher("/motion/play", String, queue_size=10)
        self.pub_evt   = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=20)
        self.pub_lock  = rospy.Publisher(self.speak_lock_ns, String, queue_size=1)
        self.pub_head_dir = rospy.Publisher("/detected_object_pose", String, queue_size=10)

        rospy.Subscriber(self.invite_topic,        String, self._cb_human_invite,  queue_size=10)
        rospy.Subscriber(self.human_show_topic,    String, self._cb_human_show,    queue_size=10)
        rospy.Subscriber(self.human_praise_topic,  String, self._cb_human_praise,  queue_size=10)
        rospy.Subscriber(self.human_pet_topic,     String, self._cb_human_pet,     queue_size=10)
        rospy.Subscriber(self.robot_show_topic,    String, self._cb_robot_show,    queue_size=5)
        rospy.Subscriber(self.robot_praise_topic,  String, self._cb_robot_praise,  queue_size=5)

        # SULK_START を拾う（いじけ開始は別ノードが出す想定）
        rospy.Subscriber("/interaction_events", InteractionEvent, self._cb_any_event, queue_size=50)

        # ASK サービス（0/1ボタン）
        self.ask = None
        if HAS_ASK:
            try:
                rospy.wait_for_service("ask_for_item", timeout=2.0)
                self.ask = rospy.ServiceProxy("ask_for_item", AskForItem)
            except Exception:
                self.ask = None

        # ===== Action Clients =====
        self.paint_cli = actionlib.SimpleActionClient("paint_stroke", PaintStrokeAction)
        self.paint_cli.wait_for_server(rospy.Duration(3.0))

        self.has_show_action = False
        self.show_cli = actionlib.SimpleActionClient("show_art", ShowArtAction)
        if self.show_cli.wait_for_server(rospy.Duration(2.0)):
            self.has_show_action = True

        # ===== Internal state =====
        self._busy = False               # 発話ロックと合わせて使用
        self._painting = False           # paint action 実行中
        self._queued = []                # 割り込みを保留
        now = time.time()
        self._last_human_ts = now       # 直近で人の入力を受けた時刻
        self._last_selfdrive_try = 0.0   # 直近自己駆動トライ時刻

        # ドライブ状態
        self.create_drive  = clamp(self.create_init,  self.drive_min, self.drive_max)
        self.connect_drive = clamp(self.connect_init, self.drive_min, self.drive_max)
        self._last_tick_ts = time.time()
        self._last_self_success_ts = 0.0

        # ドライブ更新タイマー（旧selfdrive互換の周期でもOK）
        tick_period = max(1.0/self.tick_hz, self.selfdrive_check_period_sec)
        rospy.Timer(rospy.Duration(tick_period), self._tick_drives, oneshot=False)

        rospy.loginfo("event_orchestrator: ready (auto_show=%s, auto_robot_praise=%s, show_action=%s)",
                      self.auto_show_after_paint, self.auto_robot_praise_after_show, self.has_show_action)

    # ---------- small helpers ----------
    def _lock(self, who="event"):
        self.pub_lock.publish(String(who))
        self._busy = True

    def _unlock(self):
        self.pub_lock.publish(String(""))
        self._busy = False

    def _say(self, text, emo="white"):
        if self.announce:
            self.pub_text.publish(String(text))
            self.pub_emo.publish(String(emo))

    def _log_evt(self, etype, meta=None):
        e = InteractionEvent()
        e.stamp = rospy.Time.now()
        e.event_type = etype
        e.actor_id = "event_orchestrator"
        e.target_id = "scene"
        full = dict(self.diary_meta)
        if meta: full.update(meta)
        e.meta_json = json.dumps(full, ensure_ascii=False)
        self.pub_evt.publish(e)

    # ---------- show / praise ----------
    def _robot_show(self):
        self._log_evt("ROBOT_SHOW")
        if self.has_show_action:
            self.show_cli.send_goal(ShowArtGoal(mode="pose"))
            self.show_cli.wait_for_result(rospy.Duration(5.0))
        else:
            self.pub_pose.publish(String("pose:joy"))
        self._say("見てみて！", "joy")

    def _robot_praise(self, line="すごい！いいね〜"):
        self._log_evt("ROBOT_PRAISE")
        self._say(line, "joy")
        self.pub_pose.publish(String("seq:nod"))

    # ---------- sulk ----------
    def _do_sulk(self, dur_sec: float):
        self._lock("sulk")
        try:
            self._log_evt("SULK_DO", {"dur": dur_sec})
            self._say("……", "sad")

            # 1) 首を下げる（小俯き）
            self.pub_head_dir.publish(String("center_bottom"))  # 俯く
            time.sleep(0.4)


            # 3) 少し間
            t_end = time.time() + max(0.0, dur_sec)
            while not rospy.is_shutdown() and time.time() < t_end:
                time.sleep(0.1)

            # 4) 復帰（首→正面 ／ 腕→reset）
            #self.pub_head_dir.publish(String("center_center"))  # 正面へ戻す
            self.pub_pose.publish(String("pose:reset"))

            self._say("しょんぼり", "sad")
            self.pub_pose.publish(String("seq:nod"))
            self._log_evt("SULK_END")
        finally:
            self.pub_head_dir.publish(String("center_center"))  # 正面へ戻す
            self._unlock()


    def _cb_any_event(self, evt: InteractionEvent):
        et = (evt.event_type or "").strip()
        if et == "SULK_START":
            dur = self.default_sulk_dur
            try:
                if evt.meta_json:
                    meta = json.loads(evt.meta_json)
                    if "dur" in meta:
                        dur = float(meta["dur"])
            except Exception:
                pass
            self._do_sulk(dur)
            return

        # ★ PAINT_END を拾って自己充実をバンプ
        if et == "PAINT_END":
            self._bump_create(self.bump_on_paint_end)
            self._bump_connect(0.04)  # 少しだけ関わりも上がる

    # ---------- Drive helpers ----------
    def _bump_create(self, dv): self.create_drive  = clamp(self.create_drive  + dv, self.drive_min, self.drive_max)
    def _bump_connect(self, dv):self.connect_drive = clamp(self.connect_drive + dv, self.drive_min, self.drive_max)

    def _relax(self, dt):
        dc = (self.create_base  - self.create_drive)  * self.create_relax_per_sec  * dt
        dn = (self.connect_base - self.connect_drive) * self.connect_relax_per_sec * dt
        self._bump_create(dc)
        self._bump_connect(dn)

    # ---------- human arbitration ----------
    def _touch_human(self):
        self._last_human_ts = time.time()

    def _handle_or_queue(self, kind, handler_fn, *args, **kwargs):
        """描画中の割り込み：queue / cancel のポリシーで処理する"""
        self._touch_human()
        # 関わり欲を少し上げる（人が関与してくれた）
        if kind in ("human_invite","human_show","human_praise","human_pet"):
            self._bump_connect(self.bump_on_invite if kind=="human_invite" else 0.04)

        if not self._painting:
            handler_fn(*args, **kwargs)
            return

        if not self.interrupt_enabled:
            self._queued.append((kind, handler_fn, args, kwargs))
            self._say("ちょっと待ってね…", "calm")
            return

        if self.interrupt_policy == "cancel":
            if self.use_drive_interrupt:
                # connect が高いほど中断しやすい
                p_cancel = sigmoid(self.yield_k*(self.connect_drive - self.create_drive) + self.yield_b)
            else:
                p_cancel = self.interrupt_prob
            if random.random() < p_cancel:
                try: self.paint_cli.cancel_all_goals()
                except Exception: pass
                self._painting = False
                self._say("今いくね！", "joy")
                handler_fn(*args, **kwargs)
            else:
                self._queued.append((kind, handler_fn, args, kwargs))
                self._say("少し待ってて…", "calm")
        else:
            # queue モード
            self._queued.append((kind, handler_fn, args, kwargs))
            self._say("少し待ってて…", "calm")

    def _drain_queue(self):
        for _ in range(min(3, len(self._queued))):
            kind, fn, args, kwargs = self._queued.pop(0)
            try:
                fn(*args, **kwargs)
            except Exception as e:
                rospy.logwarn("queued handler '%s' failed: %s", kind, e)

    # ---------- paint (共通) ----------
    def _start_paint_sequence(self, source="human_button"):
        """ペイント一連（人 or 自己駆動どちらからでも利用）"""
        self._lock("event_orchestrator")
        try:
            self._log_evt("PAINT_BEGIN", {"source": source})
            g = PaintStrokeGoal()  # color 等は paint_executor に委譲
            self._painting = True
            self.paint_cli.send_goal(g)

            finished = self.paint_cli.wait_for_result(rospy.Duration(self.paint_result_wait))
            self._painting = False

            if not finished:
                self.paint_cli.cancel_all_goals()
                self._say("また誘ってね。", "calm")
                self._log_evt("PAINT_TIMEOUT")
                return

            res = self.paint_cli.get_result()
            self._log_evt("PAINT_DONE")
            #self._say("できたよ！", "yellow")

            if self.auto_show_after_paint:
                self._robot_show()
                if self.auto_robot_praise_after_show:
                    self._robot_praise()

            self._drain_queue()
            self._last_self_success_ts = time.time()
        finally:
            self._unlock()

    # ---------- ASK helper ----------
    def _ask_ok(self, item_name, timeout_sec=10.0) -> bool:
        if not self.ask:
            t_end = time.time() + timeout_sec
            while not rospy.is_shutdown() and time.time() < t_end:
                time.sleep(0.1)
            return False
        try:
            resp = self.ask(AskForItemRequest(item_name=item_name))
            rospy.loginfo("ask(%s) -> %s", item_name, resp.ok)
            return bool(resp.ok)
        except Exception as e:
            rospy.logwarn("ask failed: %s", e)
            return False

    # ---------- drive tick ----------
    def _tick_drives(self, _evt):
        now = time.time()
        dt  = max(1e-3, now - self._last_tick_ts)
        self._last_tick_ts = now

        # 経時回帰
        self._relax(dt)

        # 人の入力が長くない→自己充実を少し押し上げ（自発誘因）
        if (now - self._last_human_ts) > self.nohuman_push_after:
            self._bump_create(self.bump_on_nohuman * dt)

        # 自発開始判定
        if (self.selfdrive_enabled and self.enable_self_paint and
            not self._busy and not self._painting):
            # クールダウン
            if (now - self._last_self_drive_try_ok()) < self.self_cooldown_sec:
                return
            # 閾値＆差分条件
            if self.create_drive > self.self_paint_threshold and \
               (self.create_drive - self.connect_drive) > self.self_margin_needed:
                # 旧selfdriveの最低アイドル時間・間隔も尊重
                if (now - self._last_human_ts) < self.selfdrive_min_idle_sec:
                    return
                if (now - self._last_selfdrive_try) < self.selfdrive_interval_sec:
                    return
                self._last_selfdrive_try = now
                if self.self_need_confirm:
                    self._log_evt("SELF_INVITE")
                    self._say(self.self_invite_phrase, "want")
                    ok = self._ask_ok("invite_confirm", timeout_sec=self.self_confirm_wait)
                    if ok:
                        self._log_evt("SELF_INVITE_OK")
                        self._start_paint_sequence(source="selfdrive")
                    else:
                        self._log_evt("SELF_INVITE_NG")
                        self._say("そっか、あとでにするね", "sad")
                else:
                    self._log_evt("SELF_START")
                    self._say("（…描きたいな）", "interest")
                    self.pub_pose.publish(String("seq:nod"))
                    time.sleep(0.6)
                    self._start_paint_sequence(source="selfdrive")

    def _last_self_drive_try_ok(self):
        # 「開始できた時刻」でクールダウンを取る（招待NGはカウントしない）
        return self._last_self_success_ts or 0.0

    # ---------- callbacks (human/robot) ----------
    def _cb_human_invite(self, _msg):
        # 関わり欲を押し上げる
        self._bump_connect(self.bump_on_invite)
        self._handle_or_queue("human_invite", self._start_paint_sequence, source="human_button")

    def _cb_human_show(self, _msg):
        def _do():
            self._log_evt("HUMAN_SHOW")
            time.sleep(0.6)
        self._handle_or_queue("human_show", _do)
        self._bump_connect(0.04)

    def _cb_human_praise(self, _msg):
        def _do():
            self._log_evt("HUMAN_PRAISE")
        self._handle_or_queue("human_praise", _do)
        self._bump_connect(self.bump_on_praise)

    def _cb_human_pet(self, _msg):
        def _do():
            self._log_evt("HUMAN_PET")
        self._handle_or_queue("human_pet", _do)
        self._bump_connect(self.bump_on_pet)

    def _cb_robot_show(self, _msg):
        self._handle_or_queue("robot_show", self._robot_show)

    def _cb_robot_praise(self, s: String):
        line = (s.data or "").strip() or "素敵！"
        self._handle_or_queue("robot_praise", self._robot_praise, line=line)


def main():
    EventOrchestrator()
    rospy.spin()

if __name__ == "__main__":
    main()
