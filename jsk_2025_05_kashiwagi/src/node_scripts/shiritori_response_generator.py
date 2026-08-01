#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os, rospkg
import json
import time
import re
import random

import rospy
from std_msgs.msg import String

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
        self.clear_file()
        self.latest_reply_path = os.path.join(base_dir, "latest_shiritori_reply.txt")

        # 追加：辞書ファイル
        self.words_path = os.path.join(base_dir, "shiritori_words.json")
        self.words_dict = self.load_words_dict(self.words_path)

        self.pub = rospy.Publisher("/talking_game_response", String, queue_size=10)
        rospy.Subscriber("/shiritori_word", String, self.word_callback)

        self.set_state_srv = None
        if SetKashiwagiState is not None:
            try:
                self.set_state_srv = rospy.ServiceProxy("/set_kashiwagi_state", SetKashiwagiState)
            except Exception as e:
                rospy.logwarn(f"Failed to setup /set_kashiwagi_state proxy: {e}")

        # 記録ロード（使用済み単語の復元）
        self.record = self.load_record()
        self.used_words = set(self.record.get("used_words", []))

        self.last_input = None
        self.last_reply = None

        rospy.loginfo("ShiritoriResponder node started (DICT mode).")
        rospy.spin()

    def clear_file(self):
        try:
            with open(self.record_path, "w", encoding="utf-8") as f:
                f.write("")
            rospy.loginfo(f"Cleared word file: {self.word_file}")
        except Exception as e:
            rospy.logwarn(f"Failed to clear word file: {e}")

    # ----------------- 辞書ロード -----------------
    def load_words_dict(self, path: str) -> dict:
        """
        { "あ": ["あさ", ...], "い": [...], ... } を想定。
        壊れててもノードが落ちないように空辞書で起動。
        """
        if not os.path.exists(path):
            rospy.logwarn(f"Words dict not found: {path} (start with empty dict)")
            return {}

        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                rospy.logwarn("Words dict JSON is not a dict. Use empty dict.")
                return {}

            # 値がリストでないものは除外
            cleaned = {}
            for k, v in data.items():
                if isinstance(k, str) and isinstance(v, list):
                    cleaned[k] = [str(w) for w in v if str(w).strip()]
            rospy.loginfo(f"Loaded words dict: {path} (keys={len(cleaned)})")
            return cleaned

        except Exception as e:
            rospy.logerr(f"Failed to read words dict: {e}")
            return {}

    # ★ 追加：最新reply（単語だけ）を上書き保存
    def save_latest_reply_word(self, reply_word: str) -> None:
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
            "mode": mode,
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
        w = w.strip()
        w = re.sub(r"\s+", "", w)
        w = w.strip("。、．，!！?？「」『』（）()[]【】<>《》・…―-")
        return w

    def _last_kana(self, w: str) -> str:
        w = self._strip_word(w)
        if not w:
            return ""
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

    # ----------------- 辞書ベース生成 -----------------
    def generate_reply_from_dict(self, input_word: str, max_tries: int = 200) -> str:
        """
        words_dict から「最後のかな」に対応する単語を探す。
        max_tries は候補が多い場合のランダム試行上限。
        """
        print("input word", input_word)
        last = self._last_kana(input_word)
        if not last:
            print("FFFFFFFFFFFFFFFFFFFFF")
            return ""

        candidates = self.words_dict.get(last, [])
        if not candidates:
            print("JJJJJJJJJJJJJJJJJJJJJJ")
            return ""

        # 候補が十分あるならランダムに max_tries 回試す（速い）
        # 少ないならシャッフルして総当り
        if len(candidates) <= max_tries:
            print("candidates = ", candidates)
            pool = candidates[:]
            random.shuffle(pool)
            for w in pool:
                print("w=", w)
                w = self._strip_word(w)
                if self._valid_reply(input_word, w):
                    return w
            return ""

        return ""

    # ----------------- コールバック -----------------
    def word_callback(self, msg: String):
        text = (msg.data or "").strip()
        rospy.loginfo(f"Received from /shiritori_word: {text}")

        if not text:
            rospy.logwarn("Empty input. Skip.")
            return

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

        # new: 辞書から返答を選んで publish
        input_word = self._strip_word(text)
        word = self.generate_reply_from_dict(input_word)

        # 返せない場合は空
        if not word:
            rospy.logwarn("No valid word found in dict.")
            return

        reply = input_word + "だよね。" + "うーん。" + "そうだ！" + word

        with open(f"{self.path_to_pkg}/data/tmp/tmp_response.txt",
                  "w", encoding="utf-8") as f:
            f.write(reply)

        self.save_latest_reply_word(word)
        self.pub.publish(reply)
        rospy.loginfo(f"Published reply: {reply}")

        self.last_input = input_word
        self.last_reply = word
        self.save_record(self.last_input, self.last_reply, mode="new")


if __name__ == "__main__":
    try:
        ShiritoriResponder()
    except rospy.ROSInterruptException:
        pass
