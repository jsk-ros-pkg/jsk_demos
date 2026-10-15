#!/usr/bin/env python3
import rospy
import sys, os, rospkg
from std_msgs.msg import String, Float32
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
import csv
import json
import time

class ResponseGenerator:
    def __init__(self):
        rospy.init_node("response_generator")
        rospy.sleep(1)

        # file path settings
        base_dir = os.path.dirname(__file__)
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)
        self.tsv_path = os.path.join(base_dir, "talking_game-sample.tsv")
        self.record_path = os.path.join(base_dir, "response_record.json")

        self.last_qr_distance = float('nan')
        self.qr_distance_threshold = 0.10
        self.recent_ids = {}  # id: timestamp
        self.cooldown_sec = 60
        self.cur_state = "unknown"

        # read tsv file with predefined answers
        self.qa_map = {}
        try:
            with open(self.tsv_path, encoding='utf-8') as f:
                reader = csv.DictReader(f, delimiter='\t')
                for row in reader:
                    self.qa_map[row['id']] = {
                        'question': row['question'],
                        'response': row['response']
                    }
            rospy.loginfo(f"succeeded in reading tsv: {len(self.qa_map)}")
        except Exception as e:
            rospy.logerr(f"failed to read tsv: {e}")
            return

        # read answers in the past
        self.recorded_responses = self.load_response_record()

        self.pub_response = rospy.Publisher("/talking_game_response", String, queue_size=10)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        rospy.Subscriber("/qr_distance", Float32, self.depth_update_callback, queue_size=1)
        rospy.Subscriber("/qr_data", String, self.response_callback, queue_size=1)
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)

        rospy.loginfo("Response Generator starting nodes...")
        rospy.spin()

    def load_response_record(self):
        if os.path.exists(self.record_path):
            try:
                with open(self.record_path, encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                rospy.logerr(f"failed to read response_record.json: {e}")
        return {}

    def save_response_record(self, qr_id, response):
        now = int(time.time())
        readable_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        if qr_id not in self.recorded_responses:
            self.recorded_responses[qr_id] = []

        self.recorded_responses[qr_id].append({
            "timestamp": now,                     # UNIX 時刻
            "timestamp_readable": readable_time,  # 人間が読める時刻
            "response": response
        })

        try:
            with open(self.record_path, mode='w', encoding='utf-8') as f:
                json.dump(self.recorded_responses, f, ensure_ascii=False, indent=2)
            rospy.loginfo(f"record in response_record.json {qr_id} → {response}")
        except Exception as e:
            rospy.logerr(f"failed to write in response_record.json: {e}")

    def state_callback(self, msg):
        self.cur_state = msg.data

    def depth_update_callback(self, msg):
        self.last_qr_distance = msg.data

    def response_callback(self, msg):
        qr_text = msg.data.strip()
        rospy.loginfo(f": {qr_text}")

        if self.last_qr_distance > self.qr_distance_threshold:
            rospy.logwarn("QR code is far from robot")
            return

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

        # change kashiwagi state
        try:
            resp = self.set_state_srv("talking_game:thinking_turn")
            # resp = self.set_state_srv("talking_game:speaking_turn")
            if resp.success:
                rospy.loginfo(f"state updated: {resp.message}")
            else:
                rospy.logwarn(f"no state update: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed:e {e}")
            return

        self.recent_ids[qr_text] = now

        # TSVから回答を直接取得してPublishする
        if qr_text in self.qa_map:
            reply = self.qa_map[qr_text]['response']
            rospy.loginfo(f"response from tsv: {reply}")
            
            try:
                # tmpディレクトリが存在しない場合のエラー回避
                tmp_dir = os.path.join(self.path_to_pkg, "data", "tmp")
                if not os.path.exists(tmp_dir):
                    os.makedirs(tmp_dir)
                    
                with open(os.path.join(tmp_dir, "tmp_response.txt"), "w", encoding="utf-8") as f:
                    f.write(reply)
            except Exception as e:
                rospy.logwarn(f"failed to write tmp_response.txt: {e}")

            self.pub_response.publish(reply)
            self.save_response_record(qr_text, reply)
        else:
            rospy.logwarn(f"Cannot find corresponding question and response in tsv file: {qr_text}")

if __name__ == "__main__":
    try:
        ResponseGenerator()
    except rospy.ROSInterruptException:
        pass
