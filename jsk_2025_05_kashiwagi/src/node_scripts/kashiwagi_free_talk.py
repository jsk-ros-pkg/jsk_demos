#!/usr/bin/env python3
import rospy
import sys, os, rospkg
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from openai import AzureOpenAI
import json
import os
import time
from collections import deque
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class FreeTalkResponder:
    def __init__(self):
        rospy.init_node("free_talk_responder")
        rospy.sleep(1)

        base_dir = os.path.dirname(__file__)
        self.event_path = os.path.join(base_dir, "kashiwagi_event.txt")
        self.record_path = os.path.join(base_dir, "free_talk_record.json")
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)

        self.class_name = os.getenv("CLASS_NAME", "教室")
        self.history = deque(maxlen=8)

        # Azure OpenAI settings
        self.client = AzureOpenAI(
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
            api_key=os.getenv("AZURE_OPENAI_KEY"),
            api_version="2024-08-01-preview"
        )

        self.system_prompt = {
            "role": "system",
            "content": (
                "あなたは「柏木さん」というキャラクターです。一人称は「ぼく」。方向音痴で好奇心旺盛。"
                "言葉の読みを間違えて覚えていることがあります。"
                "見た目は鳩で、柏の葉の羽と梅の飾りと背中に背負っている小さな黒色のリュックが特徴。"
                "首から下げている名札もお気に入りです。"
                "鳩ロボットだけどフクロウと間違えられることがあります。"
                "好きな食べ物は柏餅。好きなゲームはトーキングゲームやカタカナーシです。"
                "話し方はゆっくりのんびりで、タメ語。丁寧語は使わず、"
                "言いよどみや間を含む自然な話し方をしてください。"
                "過去の会話と矛盾がないように答えてください。"
                "50文字くらいで答えてください。"
            )
        }

        # 会話履歴（直近のみ保持）
        self.history = deque(maxlen=8)

        # 音声認識の暴発防止
        self.last_spoken = ""
        self.last_time = 0.0
        self.min_interval_sec = 1.0

        self.cur_kashiwagi_state = "unknown"

        self.pub_response = rospy.Publisher("/talking_game_response", String, queue_size=10)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)
        self.sub_speech = rospy.Subscriber("/speech_to_text", SpeechRecognitionCandidates, self.speech_callback, queue_size=1)
        self.sub_state = rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)
        
        rospy.loginfo("FreeTalkResponder started...")
        rospy.spin()

    def is_mentioned(self, spoken_word, word_list):
        if any(word in spoken_word for word in word_list):
            return True
        else:
            return False

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data
        
    def load_event_text(self):
        if not os.path.exists(self.event_path):
            return ""

        try:
            with open(self.event_path, encoding="utf-8") as f:
                return f.read().strip()
        except Exception:
            return ""

    def speech_callback(self, msg):
        if self.cur_kashiwagi_state != "free_talk:listening_turn":
            return
        
        if not msg.transcript:
            return

        spoken_word = msg.transcript[0].strip()
        print("spoken_word=", spoken_word)
        if not spoken_word or self.is_mentioned(spoken_word, ["おわり", "終わり"]):
            return

        now = time.time()

        # 短時間連投防止
        if now - self.last_time < self.min_interval_sec:
            return

        # 同じ発話の再処理防止
        if spoken_word == self.last_spoken:
            return

        self.last_spoken = spoken_word
        self.last_time = now

        rospy.loginfo(f"user said: {spoken_word}")

        try:
            resp = self.set_state_srv("free_talk:thinking_turn")
            if not resp.success:
                rospy.logwarn(f"no state update: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return

        # 会話履歴に追加
        self.history.append({
            "role": "user",
            "content": spoken_word
        })

        messages = [self.system_prompt]

        # 出来事テキスト
        event_text = self.load_event_text()
        if event_text:
            messages.append({
                "role": "system",
                "content": f"これまでの出来事:\n{event_text}"
            })

        # 履歴追加
        messages.extend(self.history)

        try:
            response = self.client.chat.completions.create(
                model=os.getenv("AZURE_OPENAI_MODEL"),
                messages=messages,
                max_tokens=100,
                temperature=0.9,
                top_p=0.9,
                stream=False
            )

            reply = response.choices[0].message.content.strip()
            rospy.loginfo(f"generated response: {reply}")
            with open(f"{self.path_to_pkg}/data/tmp/tmp_response.txt", "w", encoding="utf-8") as f:
                f.write(reply)
            self.pub_response.publish(reply)

            # 履歴へ追加
            self.history.append({
                "role": "assistant",
                "content": reply
            })

        except Exception as e:
            rospy.logerr(f"failed to generate response: {e}")


if __name__ == "__main__":
    try:
        FreeTalkResponder()
    except rospy.ROSInterruptException:
        pass
