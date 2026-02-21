#!/usr/bin/env python3
import rospy
import sys, os, rospkg
from std_msgs.msg import String
from openai import AzureOpenAI
import os
import json
import time
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

class KatakanaWordExplainer:
    def __init__(self):
        rospy.init_node("katakana_word_explainer")
        rospy.sleep(1)

        # --- ファイルパス設定（回答ログ保存用） ---
        base_dir = os.path.dirname(__file__)
        self.record_path = os.path.join(base_dir, "katakana_explanation_record.json")
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)
        self.set_state_srv = rospy.ServiceProxy('/set_kashiwagi_state', SetKashiwagiState)

        # 説明の履歴 { "ワード": [ {timestamp: int, explanation: str}, ... ] }
        self.explanation_record = self.load_explanation_record()

        # 直近のお題（"again" のときに使う）
        self.current_word = None

        # --- Azure OpenAI クライアント設定 ---
        self.client = AzureOpenAI(
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
            api_key=os.getenv("AZURE_OPENAI_KEY"),
            api_version="2024-08-01-preview"
        )

        # システムプロンプト
        self.system_prompt = {
            "role": "system",
            "content": (
                "あなたは「柏木さん」というキャラクターです。一人称は「ぼく」。方向音痴で好奇心旺盛。"
                "見た目は鳩で、柏の葉の羽と梅の飾りが特徴。"
                "話し方はゆっくりのんびりで、タメ語。丁寧語は使わず、言いよどみや間を含む自然な話し方をしてください。"
                "過去の回答や参考回答と矛盾がないように答えてください。"

            )
        }

        # Publisher: 説明文を出すトピック
        self.pub_explanation = rospy.Publisher(
            "/talking_game_response", String, queue_size=10
        )

        # Subscriber: カタカナ単語 or "again"
        rospy.Subscriber(
            "/katakana_theme_word", String, self.word_callback
        )

        rospy.loginfo("KatakanaWordExplainer node started.")
        rospy.spin()



    # ---------- ログ読み書き ----------
    def load_explanation_record(self):
        if os.path.exists(self.record_path):
            try:
                with open(self.record_path, encoding="utf-8") as f:
                    data = json.load(f)
                    rospy.loginfo(
                        f"Loaded explanation record: {sum(len(v) for v in data.values())} items"
                    )
                    return data
            except Exception as e:
                rospy.logerr(f"Failed to read explanation record: {e}")
        return {}

    def save_explanation_record(self, word, explanation):
        now = int(time.time())
        readable_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        if word not in self.explanation_record:
            self.explanation_record[word] = []

        self.explanation_record[word].append({
            "timestamp": now,
            "timestamp_readable": readable_time,
            "explanation": explanation
        })

        try:
            with open(self.record_path, mode="w", encoding="utf-8") as f:
                json.dump(self.explanation_record, f, ensure_ascii=False, indent=2)
            rospy.loginfo(f"Record saved: {word} → {explanation}")
        except Exception as e:
            rospy.logerr(f"Failed to write explanation record: {e}")

    # ---------- メインのコールバック ----------
    def word_callback(self, msg: String):
        try:
            resp = self.set_state_srv("katakanashi:thinking_turn")
            # resp = self.set_state_srv("talking_game:speaking_turn")
            if resp.success:
                rospy.loginfo(f"state updated: {resp.message}")
            else:
                rospy.logwarn(f"no state update: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed:e {e}")
            return
        text = msg.data.strip()
        rospy.loginfo(f"Received from /katakana_theme_word: {text}")

        if not text:
            rospy.logwarn("Received empty text. Skip.")
            return

        # "again" が来た場合は、直前のお題に対して別説明を要求
        if text.lower() == "again":
            if not self.current_word:
                rospy.logwarn('"again" was received but there is no current word yet.')
                return
            mode = "again"
            target_word = self.current_word
        else:
            # 新しいお題
            mode = "new"
            target_word = text
            # 別のお題が来たのでリセット（again の対象を更新）
            self.current_word = target_word

        # --- GPT へのプロンプト生成 ---
        if mode == "new":
            # 「最初の説明」モード：過去の説明は参照しない（リセット）
            user_prompt = (
                f"・カタカナ語「{target_word}」の意味を**カタカナ語を使わずに**説明してください。\n"
                f"・{target_word}も言ってはいけません。説明から{target_word}を当ててもらうというゲームなのです\n"
                f"・カタカナ語やカタカナを含む言葉を使っていないかどうかをちゃんと確認して、含まれていた場合は新たに回答を生成し直してください"
                f"・1文または2文だけで説明してください。\n"
                f"・まず簡単に意味を伝え、その中で必要なら少しだけ補足してください。\n"
                f"・出力は1文または2文だけにしてください。箇条書きや改行は使わないでください。"
                f"・回答に困ったときは「直訳すると〇〇」「これは〇〇に関する単語」というのはよく使うフレーズです。"
            )
        else:
            # "again" モード：これまでの説明と少し違う言い方で、1〜2文だけ
            history_list = self.explanation_record.get(target_word, [])
            history_text = ""
            # 直近3件ぐらいだけプロンプトに入れる
            for i, item in enumerate(history_list[-10:]):
                history_text += f"・これまでの説明{i+1}: {item['explanation']}\n"

            if history_text:
                user_prompt = (
                    "参考として、これまでの説明の例を示します。これらと異なる必ず新たな情報を言ってください。\n"
                    f"{history_text}"
                )
            else:
                user_prompt = ()

            user_prompt += (
                f"カタカナ語「{target_word}」について、別の言い方で1文または2文だけで説明してください。\n"
                f"・{target_word}も言ってはいけません。説明から{target_word}を当ててもらうというゲームなのです\n"
                f"・カタカナ語「{target_word}」の意味を**カタカナ語を使わずに**説明してください。\n"
                f"・カタカナ語やカタカナを含む言葉を使っていないかどうかをちゃんと確認して、含まれていた場合は新たに回答を生成し直してください"
                f"・新たな情報を追加してください。。\n"
                f"・出力は1文または2文だけにしてください。箇条書きや改行は使わないでください。\n"
            )

        try:
            response = self.client.chat.completions.create(
                model=os.getenv("AZURE_OPENAI_MODEL"),
                messages=[
                    self.system_prompt,
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=150,
                temperature=0.5,
                top_p=0.9,
                frequency_penalty=0,
                presence_penalty=0,
                stream=False
            )

            reply = response.choices[0].message.content.strip()
            rospy.loginfo(f"Generated explanation ({target_word}): {reply}")
            with open(f"{self.path_to_pkg}/data/tmp/tmp_response.txt", "w", encoding="utf-8") as f:
                f.write(reply)

            # トピックに publish
            self.pub_explanation.publish(reply)

            # ログ保存（どんな回答が生成されたかすべて保存）
            self.save_explanation_record(target_word, reply)

        except Exception as e:
            rospy.logerr(f"Failed to generate explanation with GPT: {e}")

if __name__ == "__main__":
    try:
        KatakanaWordExplainer()
    except rospy.ROSInterruptException:
        pass
