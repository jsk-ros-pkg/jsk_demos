#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
from jsk_recognition_msgs.msg import ClassificationResult
from geometry_msgs.msg import Point
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from kashiwagi_move_and_rotate_utils import MoveAndRotate

class StateAutoResetter:
    def __init__(self):
        rospy.init_node("kashiwagi_move_and_rotate_manager")
        rospy.sleep(1)
        rospy.Subscriber('/gesture_recognition/result', ClassificationResult, self.hand_pose_update_callback, queue_size=1)
        rospy.Subscriber('/gesture_recognition/hand_position', Point, self.rotate_callback, queue_size=1)
        self.cur_state = "move:finding_person"
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback)
        rospy.Subscriber("/kashiwagi_state", String, self.move_forward)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        self.latest_hand_pose = "no_hand"
        self.move_and_rotate = MoveAndRotate()

        self.nice_position_counter = 0

        rospy.loginfo("Launching Move and Rotate node ....")
        rospy.spin()

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.cur_state:
            rospy.loginfo(f"State changed: {self.cur_state} → {new_state}")
            self.cur_state = new_state

    def hand_pose_update_callback(self, msg):
        if msg.label_names:
            self.latest_hand_pose = msg.label_names[0]
        else:
            self.latest_hand_pose = "no_hand"
        print(self.latest_hand_pose, "!!!!!!!!!!!!!!!!!!!!!!!")

    def rotate_callback(self, msg):
        if self.cur_state == "move:finding_person":
            if self.latest_hand_pose == "Paper":
                # 丁度いいポジションで手をふっている人がいる
                if 300 <= msg.x <= 500:
                    print("OK")
                    self.nice_position_counter += 1

                    if self.nice_position_counter >= 5:
                        req_state = "move:approaching_person"
                        try:
                            resp = self.set_state_srv(req_state)
                            if resp.success:
                                rospy.loginfo(f"State reset successful: {resp.message}")
                            else:
                                rospy.logwarn(f"State reset failed: {resp.message}")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"Failed to call service: {e}")

                # 手をふっている人がいるけど、もうちょっと回転しないといけない
                elif msg.x < 300:
                    self.move_and_rotate.rotate_target_radian(-0.05)
                    print("AAAAAAAAAA")
                    self.nice_position_counter = 0
                elif msg.x > 500:
                    self.move_and_rotate.rotate_target_radian(0.05)
                    print("BBBBBBBBBBBBBB")
                    self.nice_position_counter = 0
                else:
                    self.nice_position_counter = 0
            # 手をふっている人が見当たらない
            else:
                print(self.latest_hand_pose)
                self.move_and_rotate.rotate_target_radian(-0.1)
                self.nice_position_counter = 0
                print("CCCCCCCCCCCC")
        else:
            pass

    def move_forward(self, msg):
        if self.cur_state == "move:approaching_person":
            self.move_and_rotate.move_forward_target_distance(0.01)

        
if __name__ == "__main__":
    try:
        StateAutoResetter()
    except rospy.ROSInterruptException:
        pass
