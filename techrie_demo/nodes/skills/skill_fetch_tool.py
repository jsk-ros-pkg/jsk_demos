#!/usr/bin/env python3
import rospy, actionlib
from techrie_demo.msg import FetchToolAction, FetchToolResult
from techrie_demo.srv import AskForItem, AskForItemRequest

def main():
    rospy.init_node('skill_fetch_tool')
    def execute(goal):
        try:
            rospy.wait_for_service('ask_for_item', timeout=3.0)
            ask = rospy.ServiceProxy('ask_for_item', AskForItem)
            res = ask(AskForItemRequest(item_name=goal.item_name))
            server.set_succeeded(FetchToolResult(received=res.ok))
        except Exception as e:
            rospy.logwarn("ask_for_item failed: %s", e)
            server.set_succeeded(FetchToolResult(received=False))
    global server
    server = actionlib.SimpleActionServer('fetch_tool', FetchToolAction, execute, auto_start=False)
    server.start()
    rospy.spin()

if __name__=='__main__':
    main()
