#!/usr/bin/env python3
import rospy, actionlib
from techrie_demo.msg import ExpressEmotionAction, ExpressEmotionResult
from std_msgs.msg import ColorRGBA, String

def main():
    rospy.init_node('skill_express_emotion')
    led_pub = rospy.Publisher('/led_rgb', ColorRGBA, queue_size=1)
    speech = rospy.Publisher('/jedy_voice', String, queue_size=1)
    colors = rospy.get_param('/colors', {})
    def execute(goal):
        kind = (goal.kind or 'JOY').upper()
        speech.publish(String(data={'THANKS':'ありがとう！','PRAISE':'すごいね！','JOY':'やったー！','SAD':'しょんぼり...'}.get(kind,'')))
        col = ColorRGBA(*colors.get('joy',[1.0,0.6,0.0,1.0]))
        led_pub.publish(col)
        rospy.sleep(1.0)
        server.set_succeeded(ExpressEmotionResult(done=True))
    global server
    server = actionlib.SimpleActionServer('express_emotion', ExpressEmotionAction, execute, auto_start=False)
    server.start()
    rospy.spin()

if __name__=='__main__':
    main()
