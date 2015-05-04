#! /usr/bin/env python
# license removed for brevity
import time
import rospy
import threading
from std_msgs.msg import Float64
from drive_recognition.msg import Int8Float64

class switch_pedal_planner:
    def __init__(self):
        self.pub = rospy.Publisher("hand_wheel/output", Float64, queue_size=10)
        rospy.Subscriber("hand_wheel/handle/input", Float64, self.callback_handle)
        rospy.Subscriber("hand_wheel/local_planner/ref_angle", Float64, self.callback_planner)
        rospy.init_node("pedal_planner_switch", anonymous=True)
        self.r = rospy.Rate(30) # 30hz
        self.is_handle = False
        self.planner_input = None
        self.planner_lock = threading.Lock()
        self.handle_input = None
        self.handle_lock = threading.Lock()

    def execute(self):
        while not rospy.is_shutdown():
            self.switch_inputs()
            self.r.sleep()

    def switch_inputs(self):
        # rospy.logwarn("%s", type(self.planner_input))
        pub_msg = Float64()
        handle_not_published_time = 5

        self.planner_lock.acquire()
        self.handle_lock.acquire()

        if self.handle_input == None:
            if self.planner_input != None:
                pub_msg.data = self.planner_input
            else:
                # No input both handle and planner
                pub_msg = None
        else:
            # is there handle?
            if self.handle_input != None and self.is_handle == False:
                rospy.loginfo("Using Controller!!")
                self.is_handle = True
                self.handle_start = time.time()

            # define output
            if self.is_handle == True:
                pub_msg.data = self.handle_input
                # not_published decision
                current_time = time.time() - self.handle_start
                if current_time > handle_not_published_time:
                    self.is_handle = False
                    self.handle_input = None
            else:
                pub_msg.data = self.planner_input

        # publish
        if pub_msg != None:
            self.pub.publish(pub_msg)

        self.planner_lock.release()
        self.handle_lock.release()

    def callback_planner(self, msg):
        with self.planner_lock:
            self.planner_input = msg.data

    def callback_handle(self, msg):
        with self.handle_lock:
            self.handle_input = msg.data

if __name__ == '__main__':
    try:
        node = switch_pedal_planner()
        node.execute()
    except rospy.ROSInterruptException: pass
