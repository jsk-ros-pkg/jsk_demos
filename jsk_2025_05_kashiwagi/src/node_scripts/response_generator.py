#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from geometry_msgs.msg import Point
from openai import AzureOpenAI
import csv
import json
import os
import time

class ResponseGenerator:
    def __init__(self):
        rospy.init_node("response_generator")
        rospy.sleep(1)

        # file path settings
        base_dir = os.path.dirname(__file__)
        self.tsv_path = os.path.join(base_dir, "talking_game.tsv")
        self.record_path = os.path.join(base_dir, "response_record.json")
        # 追加: eventテキストのファイルパス
        self.event_path = os.path.join(base_dir, "kashiwagi_event.txt")

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
                "見た目は鳩で、柏の葉の羽と梅の飾りと背中に背負っている小さな黒色のリュックが特徴"
                "好きな食べ物は柏餅。"
                "話し方はゆっくりのんびりで、タメ語。丁寧語は使わず、言いよどみや間を含む自然な話し方をしてください。"
                "過去の回答や参考回答と矛盾がないように答えてください。"
            )
        }

        self.pub_response = rospy.Publisher("/talking_game_response", String, queue_size=10)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        rospy.Subscriber("/qr_distance", Float32, self.depth_update_callback)
        rospy.Subscriber("/qr_data", String, self.response_callback)
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback)

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

    def save_response_record(self, qr_id, gpt_response):
        now = int(time.time())
        readable_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        if qr_id not in self.recorded_responses:
            self.recorded_responses[qr_id] = []

        self.recorded_responses[qr_id].append({
            "timestamp": now,                     # UNIX 時刻
            "timestamp_readable": readable_time,  # 人間が読める時刻 ← 追加
            "response": gpt_response
        })

        try:
            with open(self.record_path, mode='w', encoding='utf-8') as f:
                json.dump(self.recorded_responses, f, ensure_ascii=False, indent=2)
            rospy.loginfo(f"record in response_record.json {qr_id} → {gpt_response}")
        except Exception as e:
            rospy.logerr(f"failed to write in response_record.json: {e}")

    def load_event_text(self):
        """
        kashiwagi_event.txt からイベント文を読み込む。
        読み込めなかった場合は空文字列を返す。
        """
        if not os.path.exists(self.event_path):
            rospy.logwarn(f"event file not found: {self.event_path}")
            return ""
        try:
            with open(self.event_path, encoding='utf-8') as f:
                text = f.read().strip()
                if not text:
                    rospy.logwarn("event file is empty")
                return text
        except Exception as e:
            rospy.logerr(f"failed to read event file: {e}")
            return ""

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

        # change kashiwagi state to "talking_game:speaking_turn"
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

        # 質問と参考回答、履歴からプロンプトを作成
        if qr_text in self.qa_map:
            question = self.qa_map[qr_text]['question']
            reference = self.qa_map[qr_text]['response']
            history_list = self.recorded_responses.get(qr_text, [])

            history_text = ""
            for i, item in enumerate(history_list[-3:]):  # 直近3件のみプロンプトに含める
                history_text += f"【過去の回答{i+1}】{item['response']}\n"

            # ここで毎回 event テキストを読み込む
            event_text = self.load_event_text()

            prompt = f"""以下の質問に、柏木さんとして自然な形でタメ語で答えてください。
            - 話し方はゆっくりのんびりで、言いよどみや間を自然に入れてください。
            - 以下の「参考回答」および「過去の回答」と矛盾がないようにしてください。
            【質問】{question}
            【参考回答】{reference}
            {history_text}
            - 参考回答や過去の回答を参照しているという事実は回答中で言わないでください。
            - 以下のこれまでの出来事も参考にしてください
            【出来事】{event_text}
            - 「ふれあい」「さわだ」「澤田」は固有名詞なので、変えることなく、そのまま使ってください。ただし回答に無理にそれらの単語を入れる必要はありません。
            - 「澤田」「さわだ」の敬称は必ず「さん」でお願いします。
            - 「鳩」という言葉については「ハト」と書くようにしてください。
            - 文字カウントをして300文字に収まっているのかを確認し、文章を途中で終わらせないでください。
            """
            print(prompt)

            try:
                response = self.client.chat.completions.create(
                    model=os.getenv("AZURE_OPENAI_MODEL"),
                    messages=[
                        self.system_prompt,
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=300,
                    temperature=0.9,
                    top_p=0.9,
                    frequency_penalty=0,
                    presence_penalty=0,
                    stream=False
                )
                print(response.choices[0].message.content)
                reply = response.choices[0].message.content.strip()
                rospy.loginfo(f"generated response: {reply}")
                with open("/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/tmp/tmp_response.txt", "w", encoding="utf-8") as f:
                    f.write(reply)
                self.pub_response.publish(reply)
                self.save_response_record(qr_text, reply)

            except Exception as e:
                rospy.logerr(f"failed to generate response with GPT: {e}")
        else:
            rospy.logwarn(f"Cannot find corresponding question and response in tsv file: {qr_text}")

if __name__ == "__main__":
    try:
        ResponseGenerator()
    except rospy.ROSInterruptException:
        pass
