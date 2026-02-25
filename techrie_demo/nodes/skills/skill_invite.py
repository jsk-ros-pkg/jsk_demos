#!/usr/bin/env python3
import rospy, actionlib, json
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from techrie_demo.msg import InviteAction, InviteResult, InteractionEvent

OK_BTN = 0   # ←環境に合わせて変更 (Xbox A / PS × 等)
NG_BTN = 1   # ←環境に合わせて変更 (Xbox B / PS ○ 等)

emo_pub = rospy.Publisher('/emotion/set', String, queue_size=3)
txt_pub = rospy.Publisher('/robot_text',  String, queue_size=3)

_event_pub = None
def log_event(event_type, meta=None, actor="robot", target="human", intensity=1.0):
    """共通: InteractionEvent をpublish"""
    global _event_pub
    if _event_pub is None:
        _event_pub = rospy.Publisher('/interaction_events', InteractionEvent, queue_size=10)
        rospy.sleep(0.05)
    e = InteractionEvent()
    e.stamp = rospy.Time.now()
    e.event_type = event_type
    e.actor_id = actor
    e.target_id = target
    e.intensity = intensity
    e.meta_json = json.dumps(meta or {}, ensure_ascii=False)
    _event_pub.publish(e)

def main():
    rospy.init_node('skill_invite')
    pub_say = rospy.Publisher('/jedy_voice', String, queue_size=5)
    last_buttons = []

    def joy_cb(m: Joy):
        nonlocal last_buttons
        last_buttons = list(m.buttons)

    rospy.Subscriber('/joy', Joy, joy_cb)

    def execute(goal):
        phrase = goal.phrase or "いっしょに絵を描こう？"
        pub_say.publish(String(phrase))
        log_event("INVITE_SAID", {"phrase": phrase})

        # 10秒間、OK/NGを待つ
        timeout = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(30)
        accepted = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if last_buttons:
                if len(last_buttons) > OK_BTN and last_buttons[OK_BTN]:
                    accepted = True
                    emo_pub.publish(String('joy'))
                    txt_pub.publish(String('やった！'))
                    break
                if len(last_buttons) > NG_BTN and last_buttons[NG_BTN]:
                    accepted = False
                    emo_pub.publish(String('sad'))
                    txt_pub.publish(String('そっか…'))
                    break
            rate.sleep()

        if accepted:
            log_event("INVITE_OK")
        else:
            log_event("INVITE_NG")

        server.set_succeeded(InviteResult(accepted=accepted))

    global server
    server = actionlib.SimpleActionServer('invite', InviteAction, execute, auto_start=False)
    server.start()
    rospy.loginfo("skill_invite ready (OK_BTN=%d, NG_BTN=%d)", OK_BTN, NG_BTN)
    rospy.spin()

if __name__=='__main__':
    main()

