#!/usr/bin/env python3
import rospy, actionlib, sys
from techrie_demo.msg import InviteAction, InviteGoal, PaintStrokeAction, PaintStrokeGoal
from geometry_msgs.msg import Point

rospy.init_node("ping_actions")
cmd = sys.argv[1] if len(sys.argv)>1 else "invite"
if cmd == "invite":
    cli = actionlib.SimpleActionClient("invite", InviteAction); cli.wait_for_server()
    cli.send_goal(InviteGoal(phrase="いっしょに絵を描こう？")); cli.wait_for_result(); print(cli.get_result())
elif cmd == "paint":
    cli = actionlib.SimpleActionClient("paint_stroke", PaintStrokeAction); cli.wait_for_server()
    g = PaintStrokeGoal(color="red", start=Point(0,0,0), end=Point(0.2,0,0), pressure=0.5)
    cli.send_goal(g); cli.wait_for_result(); print(cli.get_result())
