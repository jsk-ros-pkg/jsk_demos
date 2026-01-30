#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import jaconv
from fugashi import Tagger
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class NounReadingPublisher:
    def __init__(self):
        self.in_topic = "/speech_to_text"
        self.out_topic = "/shiritori_word"

        self.word_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/src/node_scripts/latest_shiritori_reply.txt"
        self.clear_word_file()

        self.tagger = Tagger()
        self.cur_state = "unknown"

        self.pub = rospy.Publisher(self.out_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(
            self.in_topic,
            SpeechRecognitionCandidates,
            self.speech_callback,
            queue_size=10,
        )
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback)
        rospy.Subscriber("/talking_game_response", String, self.print_kashiwagi_shiritori_response)

        rospy.loginfo(f"[NounReadingPublisher] in={self.in_topic} out={self.out_topic}")
        rospy.loginfo(f"[NounReadingPublisher] word_file={self.word_file}")

    def state_callback(self, msg):
        self.cur_state = msg.data

    def print_kashiwagi_shiritori_response(self, msg):
        if self.cur_state != "shiritori:listening_turn" and self.cur_state != "shiritori:speaking_turn" and self.cur_state != "shiritori:thinking_turn":
            return
        else:
            print("kashiwagi said => ", msg.data)

    # ファイルを空にする
    def clear_word_file(self):
        try:
            with open(self.word_file, "w", encoding="utf-8") as f:
                f.write("")
            rospy.loginfo(f"Cleared word file: {self.word_file}")
        except Exception as e:
            rospy.logwarn(f"Failed to clear word file: {e}")

    def speech_callback(self, msg):
        if not hasattr(msg, "transcript") or not msg.transcript or self.cur_state != "shiritori:listening_turn":
            return

        spoken_word = (msg.transcript[0] or "").strip()
        if not spoken_word:
            return

        # 「終わり」「おわり」でファイルを空にする
        if "終わり" in spoken_word or "おわり" in spoken_word:
            self.clear_word_file()
            return

        ok, hira = self._noun_only_hiragana_reading(spoken_word)
        if not ok or not hira:
            return

        # ★追加：「ん」で終わる単語は受け付けない（publishしないで次を待つ）
        if self._ends_with_n(hira):
            rospy.loginfo(f"Blocked (ends with ん): spoken_word='{spoken_word}' reading='{hira}'")
            return

        target_initial, raw_word = self._load_last_char_with_debug()

        if target_initial is None or self._is_allowed_start(hira, target_initial):
            self.pub.publish(String(data=hira))
            rospy.loginfo(
                f"Published: spoken_word='{spoken_word}' reading='{hira}' "
                f"target='{target_initial}' from_file_word='{raw_word}'"
            )
        else:
            rospy.loginfo(
                f"Blocked: spoken_word='{spoken_word}' reading='{hira}' "
                f"target='{target_initial}' from_file_word='{raw_word}'"
            )

    def _ends_with_n(self, hira: str) -> bool:
        hira = (hira or "").strip()
        if not hira:
            return False
        return hira[-1] == "ん"

    def _load_last_char_with_debug(self):
        try:
            with open(self.word_file, "r", encoding="utf-8") as f:
                word = f.read()
            stripped = (word or "").strip()

            if not stripped:
                rospy.logwarn("word file is empty. no initial restriction.")
                return None, ""

            last_char = stripped[-1]
            rospy.loginfo(f"Loaded file word='{stripped}', last_char='{last_char}'")
            return last_char, stripped

        except Exception as e:
            rospy.logwarn(f"failed to read word file: {e}. no initial restriction.")
            return None, ""

    _small_to_big = {
        "ゃ": "や", "ゅ": "ゆ", "ょ": "よ",
        "ぁ": "あ", "ぃ": "い", "ぅ": "う", "ぇ": "え", "ぉ": "お",
        "っ": "つ", "ゎ": "わ",
    }

    def _is_allowed_start(self, hira: str, target: str) -> bool:
        if not hira:
            return False

        # 通常の一致
        if hira[0] == target:
            return True

        # target が小文字のときの特別ルール
        if target in self._small_to_big:
            big = self._small_to_big[target]
            # 小→大 (ゃ→や 等)
            if hira[0] == big:
                return True
            # 拗音許容（しゃ, きゃ, りょ など）
            if len(hira) >= 2 and hira[1] == target:
                return True

        return False

    def _noun_only_hiragana_reading(self, text: str):
        tokens = list(self.tagger(text))
        if not tokens:
            return False, ""

        for t in tokens:
            pos1 = getattr(t.feature, "pos1", None)
            if pos1 != "名詞":
                return False, ""

        kana_parts = []
        for t in tokens:
            kana = (
                getattr(t.feature, "reading", None)
                or getattr(t.feature, "kana", None)
                or getattr(t.feature, "pron", None)
                or ""
            )
            kana_parts.append(kana)

        kana = "".join(kana_parts).strip()
        if not kana:
            return True, ""

        hira = jaconv.kata2hira(kana)
        return True, hira


if __name__ == "__main__":
    rospy.init_node("noun_reading_publisher")
    node = NounReadingPublisher()
    rospy.spin()
