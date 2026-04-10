#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

TOGGLE_BTN = rospy.get_param('/allow_switch/btn', 7)  # 例: START

def main():
    rospy.init_node('allow_switch')
    pub = rospy.Publisher('/allow_paint', Bool, queue_size=1, latch=False)
    state = False
    last = 0
    def cb(m: Joy):
        nonlocal state, last
        v = int(m.buttons[TOGGLE_BTN]) if len(m.buttons) > TOGGLE_BTN else 0
        if v and not last:
            state = not state
            pub.publish(Bool(state))
            rospy.loginfo("allow_paint: %s", state)
        last = v
    rospy.Subscriber('/joy', Joy, cb, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    main()

