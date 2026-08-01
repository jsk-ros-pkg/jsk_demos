#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
import re

import rospy
import sys, os, rospkg
from std_msgs.msg import String
from openai import AzureOpenAI

from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

class ShiritoriResponder:
    """
    /shiritori_word に流れてくる「単語」を受けて、しりとりの返答（単語1つだけ）を publish するノード。
    - 返答は単語だけ（改行・句読点なし）
    - 使用済みの単語は避ける
    - "again" が来たら直前の返答をそのまま再 publish（再生成しない）
    """

    def __init__(self):
        rospy.init_node("shiritori_responder")
        rospy.sleep(1)

        base_dir = os.path.dirname(__file__)
        self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)
        self.record_path = os.path.join(base_dir, "shiritori_record.json")
        self.latest_reply_path = os.path.join(base_dir, "latest_shiritori_reply.txt")

        # 出力トピック（元コードに合わせて同じにしてます。必要なら変えてOK）
        self.pub = rospy.Publisher("/talking_game_response", String, queue_size=10)

        # 入力トピック
        rospy.Subscriber("/shiritori_word", String, self.word_callback)

        # 状態変更サービス（存在しない環境でも落ちないように）
        self.set_state_srv = None
        if SetKashiwagiState is not None:
            try:
                self.set_state_srv = rospy.ServiceProxy("/set_kashiwagi_state", SetKashiwagiState)
            except Exception as e:
                rospy.logwarn(f"Failed to setup /set_kashiwagi_state proxy: {e}")

        # Azure OpenAI クライアント
        self.client = AzureOpenAI(
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
            api_key=os.getenv("AZURE_OPENAI_KEY"),
            api_version="2024-08-01-preview"
        )

        # キャラ（必要なら変えてOK）
        self.system_prompt = {
            "role": "system",
            "content": (
                "返答は必ず“単語1つだけ”。"
                "説明文・理由・句読点・改行は禁止。"
                "しりとりなので、与えられた単語の最後の文字から始まる単語を返す。"
                "同じ単語の繰り返しは禁止。語尾が「ん」(「ン」含む)で終わる単語は禁止。"
            )
        }

        # 記録ロード（使用済み単語の復元）
        self.record = self.load_record()
        self.used_words = set(self.record.get("used_words", []))

        # again 用
        self.last_input = None
        self.last_reply = None

        rospy.loginfo("ShiritoriResponder node started.")
        rospy.spin()

    # ★ 追加：最新reply（単語だけ）を上書き保存
    def save_latest_reply_word(self, reply_word: str) -> None:
        """
        latest_shiritori_reply.txt に reply_word を1行で上書き保存する。
        """
        try:
            with open(self.latest_reply_path, "w", encoding="utf-8") as f:
                f.write(reply_word.strip() + "\n")
            rospy.loginfo(f"Latest reply saved: {reply_word} -> {self.latest_reply_path}")
        except Exception as e:
            rospy.logerr(f"Failed to write latest reply file: {e}")

        
    # ----------------- 記録 -----------------
    def load_record(self):
        if os.path.exists(self.record_path):
            try:
                with open(self.record_path, encoding="utf-8") as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        return data
            except Exception as e:
                rospy.logerr(f"Failed to read record: {e}")
        return {"used_words": [], "history": []}

    def save_record(self, input_word, reply_word, mode):
        now = int(time.time())
        readable_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        if reply_word and reply_word not in self.used_words:
            self.used_words.add(reply_word)

        self.record["used_words"] = sorted(list(self.used_words))
        self.record.setdefault("history", [])
        self.record["history"].append({
            "timestamp": now,
            "timestamp_readable": readable_time,
            "mode": mode,  # "new" or "again"
            "input": input_word,
            "reply": reply_word
        })

        try:
            with open(self.record_path, "w", encoding="utf-8") as f:
                json.dump(self.record, f, ensure_ascii=False, indent=2)
        except Exception as e:
            rospy.logerr(f"Failed to write record: {e}")

    # ----------------- しりとり補助 -----------------
    _small_to_big = str.maketrans({
        "ぁ": "あ", "ぃ": "い", "ぅ": "う", "ぇ": "え", "ぉ": "お",
        "ゃ": "や", "ゅ": "ゆ", "ょ": "よ",
        "っ": "つ",
        "ゎ": "わ",
        "ァ": "ア", "ィ": "イ", "ゥ": "ウ", "ェ": "エ", "ォ": "オ",
        "ャ": "ヤ", "ュ": "ユ", "ョ": "ヨ",
        "ッ": "ツ",
        "ヮ": "ワ",
    })

    def _strip_word(self, w: str) -> str:
        # 空白や記号をできるだけ落として「単語だけ」に寄せる
        w = w.strip()
        w = re.sub(r"\s+", "", w)
        w = w.strip("。、．，!！?？「」『』（）()[]【】<>《》・…―-")
        return w

    def _last_kana(self, w: str) -> str:
        w = self._strip_word(w)
        if not w:
            return ""
        # 末尾の長音「ー」は無視して、その前を見る
        i = len(w) - 1
        while i >= 0 and w[i] == "ー":
            i -= 1
        if i < 0:
            return ""
        ch = w[i]
        ch = ch.translate(self._small_to_big)
        return ch

    def _first_kana(self, w: str) -> str:
        w = self._strip_word(w)
        if not w:
            return ""
        ch = w[0].translate(self._small_to_big)
        return ch

    def _ends_with_n(self, w: str) -> bool:
        w = self._strip_word(w)
        if not w:
            return False
        last = w[-1]
        return last in ("ん", "ン")

    def _valid_reply(self, input_word: str, reply_word: str) -> bool:
        reply_word = self._strip_word(reply_word)
        if not reply_word:
            return False
        if "\n" in reply_word or "\r" in reply_word:
            return False
        if self._ends_with_n(reply_word):
            return False
        if reply_word in self.used_words:
            return False

        need = self._last_kana(input_word)
        got = self._first_kana(reply_word)
        if not need or not got:
            return False
        return need == got

    # ----------------- OpenAI 呼び出し -----------------
    def generate_reply(self, input_word: str, max_tries: int = 10) -> str:
        last = self._last_kana(input_word)
        used_preview = "、".join(list(sorted(self.used_words))[:200])  # 長すぎ防止（先頭だけ）
        used_count = len(self.used_words)

        base_user_prompt = (
            f"入力: {input_word}\n"
            f"しりとり返答は「{last}」から始まる単語。\n"
            f"返答は単語1つだけ。説明・句読点・改行は禁止。\n"
            f"語尾が「ん」で終わる単語は禁止。\n"
            f"使用済み単語は避ける（使用済み {used_count} 件）。\n"
            f"使用済み例（抜粋）: {used_preview}\n"
        )

        last_error = ""
        for attempt in range(1, max_tries + 1):
            user_prompt = base_user_prompt
            if last_error:
                user_prompt += f"前回NG理由: {last_error}\n必ず条件を満たす“単語1つだけ”を出して。\n"
            try:
                resp = self.client.chat.completions.create(
                    model=os.getenv("AZURE_OPENAI_MODEL"),
                    messages=[self.system_prompt, {"role": "user", "content": user_prompt}],
                    max_tokens=30,
                    temperature=0.6,
                    top_p=0.9,
                    stream=False
                )
                reply = resp.choices[0].message.content or ""
                reply = self._strip_word(reply)

                if self._valid_reply(input_word, reply):
                    return reply

                # NG理由を作る（次の試行の補助）
                if not reply:
                    last_error = "空だった"
                elif self._ends_with_n(reply):
                    last_error = "語尾が「ん」だった"
                elif reply in self.used_words:
                    last_error = "使用済みだった"
                else:
                    need = self._last_kana(input_word)
                    got = self._first_kana(reply)
                    last_error = f"頭文字が違う（必要:{need} 実際:{got}）"

            except Exception as e:
                last_error = f"APIエラー: {e}"
                rospy.logerr(f"Failed to generate reply: {e}")

        # 最後までダメなら、空返し（運用側で扱いやすいように）
        return ""

    # ----------------- コールバック -----------------
    def word_callback(self, msg: String):
        text = (msg.data or "").strip()
        rospy.loginfo(f"Received from /shiritori_word: {text}")

        if not text:
            rospy.logwarn("Empty input. Skip.")
            return

        # 状態更新（任意）
        if self.set_state_srv is not None:
            try:
                resp = self.set_state_srv("shiritori:thinking_turn")
                if hasattr(resp, "success") and resp.success:
                    rospy.loginfo(f"state updated: {getattr(resp, 'message', '')}")
            except Exception as e:
                rospy.logwarn(f"State service call failed: {e}")

        # again: 直前の返答をそのまま publish
        if text.lower() == "again":
            if not self.last_reply:
                rospy.logwarn('"again" was received but there is no last reply yet.')
                return
            self.pub.publish(self.last_reply)
            rospy.loginfo(f"Republished (again): {self.last_reply}")
            self.save_record(self.last_input, self.last_reply, mode="again")
            return
    
        # new: 生成して publish
        input_word = self._strip_word(text)
        reply = input_word + "だよね。" + "うーん。" + "そうだ！" + self.generate_reply(input_word)
        
        if not reply:
            rospy.logwarn("Could not generate a valid shiritori word.")
            return

        with open(f"{self.path_to_pkg}/data/tmp/tmp_response.txt", "w", encoding="utf-8") as f:
            f.write(reply)

        self.save_latest_reply_word(reply)

        self.pub.publish(reply)
        rospy.loginfo(f"Published reply: {reply}")

        self.last_input = input_word
        self.last_reply = reply

        self.save_record(self.last_input, self.last_reply, mode="new")


if __name__ == "__main__":
    try:
        ShiritoriResponder()
    except rospy.ROSInterruptException:
        pass
