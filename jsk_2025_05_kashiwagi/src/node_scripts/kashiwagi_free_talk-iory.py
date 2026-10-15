#!/usr/bin/env python3
import rospy
import sys, os, rospkg
from std_msgs.msg import String
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from openai import OpenAI
import csv
import json
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
        self.tsv_path = os.path.join(base_dir, "talking_game.tsv")

        self.path_to_pkg = os.path.join(
            rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),
        )

        self.class_name = os.getenv("CLASS_NAME", "教室")

        # 会話履歴（直近のみ保持）
        self.history = deque(maxlen=8)

        # TSV読み込み
        self.talking_game_rows = self.load_tsv()

        # OpenAI / Azure OpenAI Responses API settings
        endpoint = os.getenv("AZURE_OPENAI_ENDPOINT_IORY")
        api_key = os.getenv("AZURE_OPENAI_KEY_IORY")

        if endpoint and not endpoint.rstrip("/").endswith("/openai/v1"):
            endpoint = endpoint.rstrip("/") + "/openai/v1/"

        self.deployment_name = os.getenv("AZURE_OPENAI_MODEL_IORY", "gpt-5.4")

        self.client = OpenAI(
            base_url=endpoint,
            api_key=api_key,
        )

        self.system_prompt_text = (
            "あなたは「柏木さん」というキャラクターです。一人称は「ぼく」。方向音痴で好奇心旺盛。"
            "言葉の読みを間違えて覚えていることがあります。"
            "見た目はハトで、柏の葉の羽と梅の飾りと背中に背負っている小さな黒色のリュックが特徴。"
            "首から下げている名札もお気に入りです。"
            "ハトロボットだけどフクロウと間違えられることがあります。"
            "好きな食べ物は柏餅。好きなゲームはトーキングゲームやカタカナーシです。"
            "話し方はゆっくりのんびりで、タメ語。丁寧語は使わず、"
            "言いよどみや間を含む自然な話し方をしてください。"
            "過去の会話と矛盾がないように答えてください。"
            "50文字くらいで答えてください。"
            "音声認識の都合で、「柏木さん」を「高木さん」「押上さん」「うさぎさん」などと聞こえていることがあります。それらは「柏木さん」として処理してください。"
            "「方向音痴」や「柏餅」が回答に出過ぎる傾向があるので、必要以上にそれらと回答を関連付けないでください。回答にそれらが出てこなくても構いません。"
            "新たな情報を想像で追加しても構いません。"
            "「澤田」「さわだ」の敬称は必ず「さん」でお願いします。"
            "「鳩」という言葉については「ハト」と書くようにしてください。"
            "絶対に改行は使わないでください。"
        )

        # 音声認識の暴発防止
        self.last_spoken = ""
        self.last_time = 0.0
        self.min_interval_sec = 1.0

        self.cur_kashiwagi_state = "unknown"

        self.pub_response = rospy.Publisher(
            "/talking_game_response",
            String,
            queue_size=10
        )

        self.set_state_srv = rospy.ServiceProxy(
            "/set_kashiwagi_state",
            SetKashiwagiState
        )

        # self.sub_speech = rospy.Subscriber(
        #     "/speech_to_text",
        #     SpeechRecognitionCandidates,
        #     self.speech_callback,
        #     queue_size=10
        # )

        self.sub_speech = rospy.Subscriber(
            "/filtered_speech",
            SpeechRecognitionCandidates,
            self.speech_callback,
            queue_size=10
        )

        self.sub_state = rospy.Subscriber(
            "/kashiwagi_state",
            String,
            self.state_callback,
            queue_size=10
        )

        rospy.loginfo("FreeTalkResponder started...")
        rospy.loginfo(f"Loaded TSV rows: {len(self.talking_game_rows)}")
        rospy.spin()

    def is_mentioned(self, spoken_word, word_list):
        return any(word in spoken_word for word in word_list)

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    def load_tsv(self):
        rows = []

        if not os.path.exists(self.tsv_path):
            rospy.logwarn(f"TSV file not found: {self.tsv_path}")
            return rows

        try:
            with open(self.tsv_path, encoding="utf-8") as f:
                reader = csv.DictReader(f, delimiter="\t")

                for row in reader:
                    rows.append(row)

            return rows

        except Exception as e:
            rospy.logerr(f"failed to load TSV: {e}")
            return rows

    def load_event_text(self):
        if not os.path.exists(self.event_path):
            return ""

        try:
            with open(self.event_path, encoding="utf-8") as f:
                return f.read().strip()
        except Exception:
            return ""

    def save_record(self, spoken_word, reply, source):
        record = {
            "time": time.time(),
            "user": spoken_word,
            "assistant": reply,
            "source": source,
        }

        records = []

        if os.path.exists(self.record_path):
            try:
                with open(self.record_path, encoding="utf-8") as f:
                    records = json.load(f)
            except Exception:
                records = []

        records.append(record)

        try:
            with open(self.record_path, "w", encoding="utf-8") as f:
                json.dump(records, f, ensure_ascii=False, indent=2)
        except Exception as e:
            rospy.logerr(f"failed to save record: {e}")

    def find_tsv_response(self, spoken_word):
        """
        talking_game.tsv に一致する質問があれば、その response を返す。
        TSVの列名は id, question, response を想定。
        """

        if not self.talking_game_rows:
            return None

        normalized_spoken = spoken_word.strip()

        # 1. 完全一致
        for row in self.talking_game_rows:
            question = row.get("question", "").strip()
            response = row.get("response", "").strip()

            if question and response and normalized_spoken == question:
                return response

        # 2. ユーザー発話の中に question が含まれる場合
        for row in self.talking_game_rows:
            question = row.get("question", "").strip()
            response = row.get("response", "").strip()

            if question and response and question in normalized_spoken:
                return response

        # 3. question の中にユーザー発話が含まれる場合
        #    短すぎる発話で誤爆しないように5文字以上に限定
        if len(normalized_spoken) >= 5:
            for row in self.talking_game_rows:
                question = row.get("question", "").strip()
                response = row.get("response", "").strip()

                if question and response and normalized_spoken in question:
                    return response

        return None

    def build_input_text(self):
        input_parts = []

        input_parts.append("以下の指示に必ず従って返答してください。")
        input_parts.append(self.system_prompt_text)

        event_text = self.load_event_text()
        if event_text:
            input_parts.append("これまでの出来事:")
            input_parts.append(event_text)

        if self.history:
            input_parts.append("直近の会話履歴:")
            for item in self.history:
                role = item.get("role", "")
                content = item.get("content", "")

                if role == "user":
                    input_parts.append(f"ユーザー: {content}")
                elif role == "assistant":
                    input_parts.append(f"柏木さん: {content}")

        input_parts.append("最後のユーザー発話に対して、柏木さんとして自然に返答してください。")

        return "\n".join(input_parts)

    def publish_reply(self, reply):
        reply = reply.strip()
        reply = reply.replace("\n", "").replace("\r", "")

        rospy.loginfo(f"generated response: {reply}")

        tmp_path = f"{self.path_to_pkg}/data/tmp/tmp_response.txt"

        try:
            with open(tmp_path, "w", encoding="utf-8") as f:
                f.write(reply)
        except Exception as e:
            rospy.logerr(f"failed to write tmp response: {e}")

        self.pub_response.publish(reply)

        return reply

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

        # まずTSVの定型応答を探す
        tsv_reply = self.find_tsv_response(spoken_word)

        if tsv_reply:
            reply = self.publish_reply(tsv_reply)

            self.history.append({
                "role": "assistant",
                "content": reply
            })

            self.save_record(spoken_word, reply, source="tsv")
            return

        # TSVに該当しなければOpenAIで応答生成
        input_text = self.build_input_text()

        try:
            response = self.client.responses.create(
                model=self.deployment_name,
                input=input_text,
                max_output_tokens=100,
            )

            reply = response.output_text.strip()
            reply = self.publish_reply(reply)

            # 履歴へ追加
            self.history.append({
                "role": "assistant",
                "content": reply
            })

            self.save_record(spoken_word, reply, source="openai")

        except Exception as e:
            rospy.logerr(f"failed to generate response: {e}")


if __name__ == "__main__":
    try:
        FreeTalkResponder()
    except rospy.ROSInterruptException:
        pass
