#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from geometry_msgs.msg import Point
import csv
import os
import time

class ResponseGenerator:
    def __init__(self):
        rospy.init_node("response_generator")

        self.tsv_path = os.path.join(os.path.dirname(__file__), "talking_game.tsv")

        self.last_qr_distance = float('nan')
        self.qr_distance_threshold = 0.10

        self.recent_ids = {} # id: timestamp
        self.cooldown_sec = 60 # ignore the same id for cooldown_sec [seconds]

        # Read talking_game.tsv from file path
        self.qa_map = {}
        try:
            with open(self.tsv_path, encoding='utf-8') as f:
                reader = csv.DictReader(f, delimiter='\t')
                for row in reader:
                    self.qa_map[row['id']] = {
                        'question': row['question'],
                        'response': row['response']
                    }
            rospy.loginfo(f"Succeeded to load tsv file: {len(self.qa_map)}")
        except Exception as e:
            rospy.logerr(f"Failed to load tsv file: {e}")
            return

        self.pub_response = rospy.Publisher("/talking_game_response", String, queue_size=10)
        self.pub_right_eye_look_at = rospy.Publisher("/eye_display_right/look_at", Point, queue_size=10)
        self.pub_left_eye_look_at = rospy.Publisher("/eye_display_left/look_at", Point, queue_size=10)
        
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)
        rospy.Subscriber("/qr_distance", Float32, self.depth_update_callback)
        rospy.Subscriber("/qr_data", String, self.response_callback)

        rospy.loginfo("Launching Response Generator node ....")
        rospy.spin()

    def look_downside(self):
        right_gaze_point = Point()
        right_gaze_point.x = -5
        right_gaze_point.y = 10
        right_gaze_point.z = 0
        
        left_gaze_point = Point()
        left_gaze_point.x = 5
        left_gaze_point.y = 10
        left_gaze_point.z = 0
        
        self.pub_right_eye_look_at.publish(right_gaze_point)
        self.pub_left_eye_look_at.publish(left_gaze_point)

    def depth_update_callback(self, msg):
        self.last_qr_distance = msg.data
        
    def response_callback(self, msg):
        qr_text = msg.data.strip()
        rospy.loginfo(f"QR data: {qr_text}")

        # check the distance between robot and qr code
        if self.last_qr_distance > self.qr_distance_threshold:
            rospy.logwarn("QR code is far from robot")
            return

        else:
            # check if the qr code data is digit and its range
            if not qr_text.isdigit():
                rospy.logwarn("QR code data is not number")
                return
            number = int(qr_text)
            if not (1 <= number <= 100):
                rospy.logwarn("QR code data is number but out of range")
                return

            # check if the qr code is scanned within certain time
            now = time.time()
            if qr_text in self.recent_ids:
                elapsed_time = now - self.recent_ids[qr_text]
                if elapsed_time < self.cooldown_sec:
                    rospy.loginfo(f"this qr code is skipped because scanned {elapsed_time:.1f} ago")
                    return

            # change kashiwagi state to "talking_game:speaking_turn"
            try:
                req_state = "talking_game:speaking_turn"
                resp = self.set_state_srv(req_state)
                if resp.success:
                    rospy.loginfo(f"State updated: {resp.message}")
                else:
                    rospy.logwarn(f"State update failed: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            self.recent_ids[qr_text] = now
            
            # Find corresponding response from tsv file
            if qr_text in self.qa_map:
                question = self.qa_map[qr_text]['question']
                response = self.qa_map[qr_text]['response']
                rospy.loginfo(f"corresponding question: {question}")
                rospy.loginfo(f"corresponding response: {response}")
                self.look_downside()
                self.pub_response.publish(response)
            else:
                rospy.logwarn(f"Cannot find corresponding question and response in tsv file: {qr_text}")

if __name__ == "__main__":
    try:
        ResponseGenerator()
    except rospy.ROSInterruptException:
        pass
