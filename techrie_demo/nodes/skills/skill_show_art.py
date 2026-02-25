#!/usr/bin/env python3
import rospy, actionlib
from techrie_demo.msg import ShowArtAction, ShowArtResult

def main():
    rospy.init_node('skill_show_art')
    def execute(goal):
        rospy.loginfo("Showing art with mode=%s", goal.mode)
        rospy.sleep(1.5)
        server.set_succeeded(ShowArtResult(done=True))
    global server
    server = actionlib.SimpleActionServer('show_art', ShowArtAction, execute, auto_start=False)
    server.start()
    rospy.spin()

if __name__=='__main__':
    main()
