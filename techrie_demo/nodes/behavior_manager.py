#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, smach, smach_ros
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import json

# msgs
from techrie_demo.msg import (
    DesireState,
    InviteAction, InviteGoal,
    ExpressEmotionAction, ExpressEmotionGoal,
    ShowArtAction, ShowArtGoal,
    PaintStrokeAction, PaintStrokeGoal,
    InteractionEvent,
)
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String as S

# ==== globals (callbacksで更新) ====
_last_policy = None
_last_invite_until = rospy.Time(0)
_allow_paint = False
_system_started = False  # ← /system/started が True になるまで停止
_started_at = rospy.Time(0)
_recent_praise = False
_recent_praise = False
_recent_pet = False
_recent_food = False
_recent_show = False
_sulk_until = rospy.Time(0)   # ← 追加：いじけ継続の締め切り

def _pol_cb(m: S):
    global _last_policy
    _last_policy = (m.data or "").strip()

def _evt_cb(e: InteractionEvent):
    """イベント処理：INVITEクールダウンと いじけ(SULK)の受付"""
    global _last_invite_until, _sulk_until
    et = getattr(e, "event_type", "")
    if et in ("INVITE_OK", "INVITE_NG"):
        cd = float(rospy.get_param("~invite_cooldown", 8.0))
        _last_invite_until = rospy.Time.now() + rospy.Duration(cd)
        return

    if et == "SULK_START":
        # meta_json = {"dur": 秒}
        dur = 12.0
        try:
            meta = json.loads(getattr(e, "meta_json", "") or "{}")
            dur = float(meta.get("dur", dur))
        except Exception:
            pass
        _sulk_until = rospy.Time.now() + rospy.Duration(dur)
        rospy.loginfo("behavior_manager: SULK latched for %.1fs", dur)

def _allow_paint_cb(m: Bool):
    global _allow_paint
    _allow_paint = bool(m.data)

def _started_cb(m: Bool):
    global _system_started, _started_at
    prev = _system_started
    _system_started = bool(m.data)
    if _system_started and not prev:
        _started_at = rospy.Time.now()  # 起動時刻を記録

def _flag_once_setter(name, duration=0.8):
    def _set(_):
        globals()[name] = True
        # duration後に自動クリア（多重押しでも最新だけ残る）
        rospy.Timer(rospy.Duration(duration), lambda _ : globals().__setitem__(name, False), oneshot=True)
    return _set    

# ==== States ====
class TopState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=["observe", "invite", "paint", "show", "eat", "idle", "praise","pet","observe_human_show","sulk"]
        )
        self._desire = DesireState()
        rospy.Subscriber("/desire/state", DesireState, self._cb, queue_size=5)
        rospy.Subscriber("/policy/next_action", S, _pol_cb, queue_size=1)
        rospy.Subscriber("/interaction_events", InteractionEvent, _evt_cb, queue_size=50)
        rospy.Subscriber("/allow_paint", Bool, _allow_paint_cb, queue_size=1)
        rospy.Subscriber("/system/started", Bool, _started_cb, queue_size=1)
        # human inputs (edge-trigger)
        rospy.Subscriber('/human/praise',    S, _flag_once_setter('_recent_praise'), queue_size=10)
        rospy.Subscriber('/human/pet',       S, _flag_once_setter('_recent_pet'),    queue_size=10)
        rospy.Subscriber('/human/offer_food',S, _flag_once_setter('_recent_food'),   queue_size=10)
        rospy.Subscriber('/human/show_art',  S, _flag_once_setter('_recent_show'),   queue_size=10)

        # ★ 追加：直近に選んだ時刻を記録（同じ状態への再突入を抑制）
        self._last_exec_sec = {}  # { state_name: epoch_sec }

    def _cb(self, m: DesireState):
        self._desire = m

    def _max_desire_label(self) -> str:
        scores = {
            "invite": float(self._desire.want_with_people),
            "paint":  float(self._desire.want_paint),
            "show":   float(self._desire.want_show),
            "eat":    float(self._desire.want_eat),
            "idle":   float(self._desire.want_idle),
            "observe": 0.1,
        }
        return max(scores, key=scores.get)

    def execute(self, _):
        period = float(rospy.get_param("~top_period", 1.0))
        min_iv = float(rospy.get_param("~min_state_interval_sec", 120.0))  # ★ 既定 120s
        rospy.sleep(period)

        if not _system_started:
            return "idle"

        # 人の入力は最優先で即遷移
        if _recent_praise: return "praise"
        if _recent_pet:    return "pet"
        if _recent_food:   return "eat"
        if _recent_show:   return "observe_human_show"

        # === ここから追加: スタート猶予（invite等をしばらく抑止） ===
        grace = float(rospy.get_param("~start_grace_sec", 10.0))  # 例: 10秒
        if _started_at != rospy.Time(0):
            dt = (rospy.Time.now() - _started_at).to_sec()
            if dt < grace:
                return "idle"

        # ★ いじけラッチが生きていたら最優先で SULK
        if rospy.Time.now() < _sulk_until:
            return "sulk"


        # 政策 or desire
        if _last_policy in ("observe", "invite", "paint", "show", "eat", "idle","sulk"):
            choice = _last_policy
        else:
            choice = self._max_desire_label()

        # 招待のクールダウン
        if choice == "invite" and rospy.Time.now() < _last_invite_until:
            choice = "idle"

        # PAINT 許可ゲート
        if choice == "paint" and not _allow_paint:
            choice = "idle"

        # ★ 同じ状態への再突入抑制（invite/paint/show/eat を主に抑制）
        tracked = ("invite","paint","show","eat","sulk")
        if choice in tracked:
            now = rospy.Time.now().to_sec()
            last = self._last_exec_sec.get(choice, 0.0)
            if (now - last) < min_iv:
                return "idle"  # 安全に idle で回す
            self._last_exec_sec[choice] = now

        return choice


class InviteState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["ok", "ng", "observe"])
        self.cli = SimpleActionClient("invite", InviteAction)
        self.cli.wait_for_server(rospy.Duration(2.0))

    def execute(self, _):
        goal = InviteGoal(phrase=rospy.get_param("invite_phrase", "いっしょに絵を描こう？"))
        self.cli.send_goal(goal)
        self.cli.wait_for_result(rospy.Duration(8.0))
        res = self.cli.get_result()
        return "ok" if (res and getattr(res, "accepted", False)) else "observe"


class ShowState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"])
        self.cli = SimpleActionClient("show_art", ShowArtAction)
        self.cli.wait_for_server(rospy.Duration(2.0))
        self.emo_pub = rospy.Publisher('/emotion/set', S, queue_size=3)
        self.txt_pub = rospy.Publisher('/robot_text',  S, queue_size=3)
        # 追加：褒められ待ち
        self._got_praise = False
        rospy.Subscriber('/human/praise', S, self._praise_cb, queue_size=10)
        # 追加：連発防止
        self._cooldown_until = rospy.Time(0)

    def _praise_cb(self, _m:S):
        self._got_praise = True

    def execute(self, _):
        now = rospy.Time.now()
        cd = float(rospy.get_param("~show_cooldown_sec", 5.0))       # 次回までのクールダウン
        wait_sec = float(rospy.get_param("~show_wait_praise_sec", 6.0))  # 褒められ待ち時間

        # クールダウン中は静かに戻る
        if now < self._cooldown_until:
            return "done"

        # 1回だけ見せる
        self.cli.send_goal(ShowArtGoal(mode="pose"))
        self.cli.wait_for_result(rospy.Duration(3.0))
        self.emo_pub.publish('joy')
        self.txt_pub.publish('見てみて！')

        # 褒められ待ち
        self._got_praise = False
        t_end = rospy.Time.now() + rospy.Duration(wait_sec)
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            if self._got_praise:
                self.emo_pub.publish('joy')
                self.txt_pub.publish('ありがとう!')
                rospy.sleep(1.0)
                break
            r.sleep()
            if not self._got_praise:
                self.emo_pub.publish(S('sad'))
                rospy.sleep(0.6)


        # 連発防止
        self._cooldown_until = rospy.Time.now() + rospy.Duration(cd)
        return "done"


class PaintState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "observe"])
        self.cli = SimpleActionClient("paint_stroke", PaintStrokeAction)
        self.cli.wait_for_server(rospy.Duration(2.0))
        # 演出（任意）
        self.em = SimpleActionClient("express_emotion", ExpressEmotionAction)
        self.sh = SimpleActionClient("show_art", ShowArtAction)
        self.em.wait_for_server(rospy.Duration(2.0))
        self.sh.wait_for_server(rospy.Duration(2.0))

    def execute(self, _):
        g = PaintStrokeGoal(color="", start=Point(0,0,0), end=Point(0.10,0,0), pressure=0.5)
        self.cli.send_goal(g)

        # 待ち時間は十分長く
        timeout_sec = rospy.get_param("~paint_action_timeout", 120.0)
        finished = self.cli.wait_for_result(rospy.Duration(timeout_sec))

        # ここが重要: 成功/失敗を get_state で判定
        state = self.cli.get_state()
        succeeded = (finished and state == GoalStatus.SUCCEEDED)

        do_celebrate = rospy.get_param("~celebrate_after_paint", True)
        if succeeded and do_celebrate:
            self.em.send_goal(ExpressEmotionGoal(kind="JOY", intensity=1.0))
            self.em.wait_for_result(rospy.Duration(3.0))
            self.sh.send_goal(ShowArtGoal(mode="pose"))
            self.sh.wait_for_result(rospy.Duration(3.0))

        return "done" if succeeded else "observe"


class PraiseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        # 表現ハブへ
        self.emo_pub = rospy.Publisher('/emotion/set', S, queue_size=3)
        self.txt_pub = rospy.Publisher('/robot_text',  S, queue_size=3)
        self._got = False
        rospy.Subscriber('/human/praise', S, self._cb, queue_size=10)

    def _cb(self, _m:S):
        self._got = True

    def execute(self, _):
        # 直近でpraiseが来ていなければ即戻る
        if not self._got:
            return 'done'
        self._got = False
        # 反応
        self.emo_pub.publish('joy')
        self.txt_pub.publish('ありがとう!')
        rospy.sleep(1.2)
        return 'done'

class PetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.emo_pub = rospy.Publisher('/emotion/set', S, queue_size=3)
        self.txt_pub = rospy.Publisher('/robot_text',  S, queue_size=3)
    def execute(self, _):
        self.emo_pub.publish(S('calm'))
        self.txt_pub.publish(S('なでなで'))
        rospy.sleep(1.0)
        return 'done'

class EatState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.emo_pub = rospy.Publisher('/emotion/set', S, queue_size=3)
        self.txt_pub = rospy.Publisher('/robot_text',  S, queue_size=3)
    def execute(self, _):
        self.emo_pub.publish(S('joy'))
        self.txt_pub.publish(S('食べたいよ'))
        rospy.sleep(1.2)
        return 'done'

class ObserveHumanShowState(smach.State):
    """人が作品を見せてくれた時のリアクション"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.emo_pub = rospy.Publisher('/emotion/set', S, queue_size=3)
        self.txt_pub = rospy.Publisher('/robot_text',  S, queue_size=3)
    def execute(self, _):
        # ここは軽い表現だけ（必要なら show_art アクションを呼んでもOK）
        self.emo_pub.publish(S('interest'))
        self.txt_pub.publish(S('わぁ！それいいね〜'))
        rospy.sleep(1.0)
        self.emo_pub.publish(S('joy'))
        rospy.sleep(0.3)
        return 'done'

class SulkState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"])
        self.txt = rospy.Publisher("/robot_text", S, queue_size=3)
        self.em  = rospy.Publisher("/emotion/set", S, queue_size=3)
        self.mo  = rospy.Publisher("/motion/play", S, queue_size=3)
    def execute(self, _):
        # いじけ演出（watchdog 側からも出すが、ここでも保険で一回）
        self.txt.publish(S("……しょんぼり"))
        self.em.publish(S("sad"))
        self.mo.publish(S("seq:shake"))
        rospy.sleep(2.0)
        return "done"


def main():
    rospy.init_node("behavior_manager")
    sm = smach.StateMachine(outcomes=["END"])
    with sm:
        # TOP：paintはPAINTへ、inviteはINVITEへ
        smach.StateMachine.add(
            "TOP",
            TopState(),
            transitions={
                "observe": "TOP",
                "invite":  "INVITE",
                "paint":   "PAINT",
                "show":    "SHOW",
                "eat":     "EAT",
                "idle":    "TOP",
                'praise':'PRAISE',
                'pet':'PET',
                "observe_human_show": "OBSERVE_HUMAN_SHOW",
                "sulk": "SULK"
            },
        )
        # INVITE：okでPAINTへ
        smach.StateMachine.add(
            "INVITE",
            InviteState(),
            #transitions={"ok": "PAINT", "ng": "TOP", "observe": "TOP"},
            transitions={"ok": "PAINT", "ng": "TOP", "observe": "TOP"},
        )
        smach.StateMachine.add("SHOW", ShowState(), transitions={"done": "TOP"})
        smach.StateMachine.add("PAINT", PaintState(), transitions={"done": "SHOW", "observe": "TOP"})
        smach.StateMachine.add('PRAISE', PraiseState(), transitions={'done':'TOP'})
        smach.StateMachine.add('PET',    PetState(),    transitions={'done':'TOP'})
        smach.StateMachine.add('EAT',    EatState(),    transitions={'done':'TOP'})
        smach.StateMachine.add('OBSERVE_HUMAN_SHOW', ObserveHumanShowState(), transitions={'done':'TOP'})
        smach.StateMachine.add("SULK", SulkState(), transitions={"done": "TOP"})

    sis = smach_ros.IntrospectionServer("techrie_demo_smach", sm, "/SM_ROOT")
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()

