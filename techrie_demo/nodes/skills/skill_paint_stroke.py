#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, actionlib, json
from techrie_demo.msg import (
    PaintStrokeAction, PaintStrokeFeedback, PaintStrokeResult, InteractionEvent
)
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from std_msgs.msg import ColorRGBA, String as S, Bool

emo_pub = rospy.Publisher('/emotion/set', S, queue_size=3)
txt_pub = rospy.Publisher('/robot_text',  S, queue_size=3)

# ========== 共通: InteractionEvent をpublish ==========
_evt_pub = None
def log_event(event_type, meta=None, actor="robot", target="human", intensity=1.0):
    global _evt_pub
    if _evt_pub is None:
        _evt_pub = rospy.Publisher('/interaction_events', InteractionEvent, queue_size=10)
        rospy.sleep(0.02)
    e = InteractionEvent()
    e.stamp = rospy.Time.now()
    e.event_type = event_type
    e.actor_id = actor
    e.target_id = target
    e.intensity = intensity
    e.meta_json = json.dumps(meta or {}, ensure_ascii=False)
    _evt_pub.publish(e)

# ========== グローバル状態 ==========
_current_color = "red"
_motion_done = False

def _color_cb(m: S):
    global _current_color
    _current_color = m.data or "red"

def _motion_done_cb(m: Bool):
    global _motion_done
    _motion_done = bool(m.data)

def main():
    rospy.init_node('skill_paint_stroke')

    # パラメータ
    use_motion_done = rospy.get_param('~use_motion_done', True)
    steps          = rospy.get_param('~steps', 20)          # 直線補間の分割数
    step_hz        = rospy.get_param('~step_hz', 20.0)      # 1ステップの送出周波数
    step_sleep     = 1.0 / max(1e-3, step_hz)

    # I/O
    #move_pub = rospy.Publisher('/arm/move_tip', Point, queue_size=20)
    motion_pub = rospy.Publisher('/paint_motion', S, queue_size=10)
    led_pub  = rospy.Publisher('/led_rgb', ColorRGBA, queue_size=1)
    rospy.Subscriber('/paint_color', S, _color_cb, queue_size=1)
    rospy.Subscriber('/motion_done', Bool, _motion_done_cb, queue_size=5)

    # ペン上下（legacy_bridge が reach/straight を投げてくれる）
    pen_cli = None
    try:
        rospy.wait_for_service('/arm/pen_down', timeout=1.0)
        pen_cli = rospy.ServiceProxy('/arm/pen_down', SetBool)
    except rospy.ROSException:
        rospy.logwarn("pen_down service not available; drawing without pen control.")

    def wait_motion_done(timeout_sec=10):
        """既存robot_behaviorの /motion_done を軽く待つ（無ければスキップ）"""
        if not use_motion_done:
            return
        global _motion_done
        _motion_done = False
        t_end = rospy.Time.now() + rospy.Duration(timeout_sec)
        r = rospy.Rate(200)
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            if _motion_done:
                break
            r.sleep()

    def execute(goal):
        color = goal.color if goal.color else _current_color
        
        # LED表示（任意）
        led_pub.publish(ColorRGBA(0.2, 0.6, 1.0, 0.8))
        
        # PAINT_STARTイベント
        log_event("PAINT_START", {"color": color})

        # --- 描く前（興味） ---
        emo_pub.publish(S('interest'))
        txt_pub.publish(S('描いてみるね！'))

        # モーション名決定（例: jedy_paint_action.json 内で定義したキー）
        motion_name = f"paint_{color}" ##あとで変更
        motion_name = "reach"
        rospy.loginfo(f"Executing motion: {motion_name}")
        #motion_pub.publish(S(motion_name))

        # 必要なら完了待ち
        wait_motion_done(5.0)

        # --- 成功後（喜び） ---
        emo_pub.publish(S('joy'))
        txt_pub.publish(S('できた！'))

        # PAINT_ENDイベント
        stroke_id = str(rospy.Time.now().to_nsec())
        log_event("PAINT_END", {"color": color, "stroke_id": stroke_id})

        server.set_succeeded(PaintStrokeResult(done=True, stroke_id=stroke_id))


    global server
    server = actionlib.SimpleActionServer('paint_stroke', PaintStrokeAction, execute, auto_start=False)
    server.start()
    rospy.loginfo("skill_paint_stroke ready. use_motion_done=%s steps=%d step_hz=%.1f",
                  use_motion_done, steps, step_hz)
    rospy.spin()

if __name__ == '__main__':
    main()

