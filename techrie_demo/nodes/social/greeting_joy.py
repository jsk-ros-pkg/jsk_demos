#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time, json
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from techrie_demo.msg import InteractionEvent
from kxr_controller.msg import ServoOnOff
HELLO_BTN = 27
BYE_BTN   = 63
DOUBLE_TAP_WINDOW = 3  # 秒以内に2回押したら有効

last_press_time = {}
press_count = {}
started = False  # 27ボタンが2回押されるまで False

system_started_pub = rospy.Publisher("/system/started", Bool, queue_size=1, latch=True)
servo_pub = rospy.Publisher("/servo_on_off", ServoOnOff, queue_size=10, latch=True)

def publish_event(evtype, meta=None):
    pub = rospy.Publisher("/interaction_events", InteractionEvent, queue_size=10)
    rospy.sleep(0.02)
    e = InteractionEvent()
    e.stamp = rospy.Time.now()
    e.event_type = evtype
    e.actor_id = "human"
    e.target_id = "robot"
    e.intensity = 1.0
    e.meta_json = json.dumps(meta or {}, ensure_ascii=False)
    pub.publish(e)

def handle_greeting(greet_type):
    emo_pub.publish(String("joy" if greet_type == "hello" else "calm"))
    if greet_type == "hello":
        txt_pub.publish(String("こんにちは！"))
        allow_pub.publish(Bool(True))
        publish_event("GREETING_HELLO", {"source": "joy"})
    else:
        txt_pub.publish(String("おやすみ〜"))
        msg = ServoOnOff()
        msg.joint_names = [""]      # robot_behavior 側で全OFFを受ける実装
        msg.servo_on_states = [False]
        servo_pub.publish(msg)
        allow_pub.publish(Bool(False))
        
        publish_event("GREETING_GOODNIGHT", {"source": "joy"})

def cb_joy(msg: Joy):
    global started
    now = time.time()
    for btn_id, greet_type in [(HELLO_BTN, "hello"), (BYE_BTN, "bye")]:
        pressed = (btn_id < len(msg.buttons) and msg.buttons[btn_id] == 1)
        if pressed:
            last_t = last_press_time.get(btn_id, 0)
            cnt = press_count.get(btn_id, 0) + 1
            press_count[btn_id] = cnt
            last_press_time[btn_id] = now
            # 2回目かつ間隔が短い
            if cnt == 2 and (now - last_t) <= DOUBLE_TAP_WINDOW:
                press_count[btn_id] = 0
                if btn_id == HELLO_BTN:
                    if not started:
                        started = True
                        handle_greeting("hello")
                        system_started_pub.publish(Bool(True))
                        rospy.loginfo("🎯 Start signal received. Entering main behavior loop.")
                elif btn_id == BYE_BTN:
                    handle_greeting("bye")
                    system_started_pub.publish(Bool(False))
                    rospy.loginfo("👋 Shutdown signal received. Exiting...")
                    rospy.signal_shutdown("Goodnight button pressed")
        else:
            # ボタン離してから一定時間過ぎたらカウントリセット
            if (now - last_press_time.get(btn_id, 0)) > DOUBLE_TAP_WINDOW:
                press_count[btn_id] = 0

if __name__ == "__main__":
    rospy.init_node("greeting_joy")
    emo_pub   = rospy.Publisher("/emotion/set", String, queue_size=3)
    txt_pub   = rospy.Publisher("/robot_text", String, queue_size=3)
    allow_pub = rospy.Publisher("/allow_paint", Bool, queue_size=1, latch=True)

    rospy.Subscriber("/joy", Joy, cb_joy, queue_size=20)
    rospy.loginfo("greeting_joy ready (double-tap btn%d=hello to start, btn%d=bye to exit)",
                  HELLO_BTN, BYE_BTN)
    rospy.spin()


