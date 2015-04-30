#! /usr/bin/env python
# license removed for brevity
import time
import rospy
import threading
from std_msgs.msg import Float64

class switch_handle_obstacle:
    def __init__(self):
        self.pub = rospy.Publisher("brake_pedal/output", Float64, queue_size=10)
        rospy.Subscriber("brake_pedal/handle/input", Float64, self.callback_handle)
        rospy.Subscriber("brake_pedal/obstacle/input", Float64, self.callback_obstacle)
        rospy.init_node("handle_obstacle_switch", anonymous=True)
        self.r = rospy.Rate(30) # 30hz
        self.is_obstacle = False
        self.handle_input = None
        self.handle_lock = threading.Lock()
        self.obstacle_input = None
        self.obstacle_lock = threading.Lock()

    def execute(self):
        while not rospy.is_shutdown():
            self.switch_inputs()
            self.r.sleep()

    def switch_inputs(self):
        pub_msg = Float64()
        obstacle_resume_time = 2.5 # after 2.5sec obstacle detection brake is unlocked

        self.handle_lock.acquire()
        self.obstacle_lock.acquire()

        if self.obstacle_input == None:
            if self.handle_input != None:
                pub_msg.data = self.handle_input
            else:
                # No input both obstacle and handle
                pub_msg = None
        else:
            # is there obstacle?
            if self.obstacle_input > 0.0 and self.is_obstacle == False:
                rospy.loginfo("EMERGENCY BRAKE!!")
                self.is_obstacle = True
                self.obstacle_start = time.time()

            # define output
            if self.is_obstacle == True:
                pub_msg.data = self.obstacle_input
                # resume decision
                current_time = time.time() - self.obstacle_start
                if current_time > obstacle_resume_time:
                    self.is_obstacle = False
                    # obstacle input should be reset because obstacle_detector does not output when there is no obstacle
                    self.obstacle_input = None
            else:
                pub_msg.data = self.handle_input

        # publish
        if pub_msg != None:
            self.pub.publish(pub_msg)

        self.handle_lock.release()
        self.obstacle_lock.release()

    def callback_handle(self, msg):
        with self.handle_lock:
            self.handle_input = msg.data

    def callback_obstacle(self, msg):
        with self.obstacle_lock:
            self.obstacle_input = msg.data

if __name__ == '__main__':
    try:
        node = switch_handle_obstacle()
        node.execute()
    except rospy.ROSInterruptException: pass
