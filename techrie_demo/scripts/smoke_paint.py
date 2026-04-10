#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, actionlib, time, os, glob
from std_msgs.msg import String as S
from geometry_msgs.msg import Point
from techrie_demo.msg import PaintStrokeAction, PaintStrokeGoal, InteractionEvent

events = []
def ev_cb(m: InteractionEvent):
    events.append((m.event_type, m.meta_json, m.stamp.to_sec()))

def latest_diary_path():
    d = os.path.expanduser("~/.ros/techrie_demo/diary")
    if not os.path.isdir(d): return None
    files = sorted(glob.glob(os.path.join(d, "*.md")))
    return files[-1] if files else None

def main():
    rospy.init_node("smoke_paint", anonymous=True)

    # 1) 色設定（red→blue に変えてみる）
    pub_color = rospy.Publisher("/paint_color", S, queue_size=1, latch=True)
    rospy.sleep(0.2)
    pub_color.publish(S("blue"))

    # 2) イベント購読開始
    rospy.Subscriber("/interaction_events", InteractionEvent, ev_cb, queue_size=50)
    rospy.sleep(0.4)  # ★ 追加：購読確立待ち

    # 3) 1本ストローク送る
    cli = actionlib.SimpleActionClient("paint_stroke", PaintStrokeAction)
    print("waiting for paint_stroke...")
    assert cli.wait_for_server(rospy.Duration(5.0)), "paint_stroke action not ready"

    g = PaintStrokeGoal(color="", start=Point(0.0, 0.0, 0.0), end=Point(0.05, 0.0, 0.0), pressure=0.5)
    t0 = time.time()
    cli.send_goal(g)
    ok = cli.wait_for_result(rospy.Duration(12.0))
    res = cli.get_result()
    print("goal_done:", ok, "result:", res)

    # 4) イベント到着を少し待つ
    rospy.sleep(0.5)
    got_start = any(e[0]=="PAINT_START" for e in events)
    got_end   = any(e[0]=="PAINT_END"   for e in events)
    print("events:", events[-4:])

    # 5) 日記ファイル更新確認
    diary = latest_diary_path()
    diary_ok = False
    if diary and os.path.exists(diary):
        mtime = os.path.getmtime(diary)
        diary_ok = (mtime > t0 - 5)  # テスト開始直前より新しければOK

    print("\n=== SMOKE RESULT ===")
    print("paint_stroke:", "OK" if ok else "NG")
    print("events PAINT_START/END:", "OK" if (got_start and got_end) else "NG")
    print("diary updated:", diary, "->", "OK" if diary_ok else "NG")

    if ok and got_start and got_end and diary_ok:
        print("\nALL GREEN ✅  1本描いて、イベントと日記に残りました。")
    else:
        print("\n❌ どれかが欠けています。/rosoutログや /interaction_events を確認してください。")

if __name__ == "__main__":
    main()
