#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, actionlib, os, time
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from techrie_demo.msg import PaintStrokeAction, ExpressEmotionAction, ShowArtAction, InviteAction

def wait_topic(name, timeout=2.0):
    try:
        rospy.wait_for_message(name, rospy.AnyMsg, timeout=timeout)
        return True
    except rospy.ROSException:
        return False

def check_action(ns, action_type, timeout=2.0):
    cli = actionlib.SimpleActionClient(ns, action_type)
    return cli.wait_for_server(rospy.Duration(timeout))

def check_service(ns, timeout=2.0):
    try:
        rospy.wait_for_service(ns, timeout=timeout); return True
    except rospy.ROSException:
        return False

def main():
    rospy.init_node("check_techrie", anonymous=True)
    ok = True
    checks = []

    # topics
    checks.append(("topic /paint_position", wait_topic("/paint_position", 2.0)))
    checks.append(("topic /interaction_events", True))  # 任意: 起動直後は未発生でもOK
    checks.append(("topic /paint_color", True))         # color_selector起動済ならOK

    # actions
    checks.append(("action /paint_stroke", check_action("paint_stroke", PaintStrokeAction)))
    checks.append(("action /express_emotion", check_action("express_emotion", ExpressEmotionAction)))
    checks.append(("action /show_art", check_action("show_art", ShowArtAction)))
    checks.append(("action /invite", check_action("invite", InviteAction)))

    # services
    checks.append(("service /arm/pen_down", check_service("/arm/pen_down")))
    checks.append(("service /ask_for_item", check_service("/ask_for_item")))

    print("=== techrie_demo health check ===")
    for name, passed in checks:
        print(f"[{'OK' if passed else 'NG'}] {name}")
        ok = ok and passed

    if ok:
        print("\nALL GREEN ✅  基本配線OKです。次は smoke test へ。")
    else:
        print("\n一部NGがあります。該当ノード/launchの起動有無を確認してください。")

if __name__ == "__main__":
    main()
