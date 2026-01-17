#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import jaconv
from fugashi import Tagger
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class NounReadingPublisher:
    """
    /speech_to_text を購読し、名詞だけなら読みを取得。
    ファイルに書かれた単語の「最後の文字」を毎回読み込み、
    それと読み(ひらがな)の先頭が一致すれば /shiritori_word に publish。
    ファイルが無い/空なら制限なし。
    """

    def __init__(self):
        self.in_topic = "/speech_to_text"
        self.out_topic = "/shiritori_word"

        self.word_file = "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/src/node_scripts/latest_shiritori_reply.txt"

        self.tagger = Tagger()

        self.pub = rospy.Publisher(self.out_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(
            self.in_topic,
            SpeechRecognitionCandidates,
            self.speech_callback,
            queue_size=10,
        )

        rospy.loginfo(f"[NounReadingPublisher] in={self.in_topic} out={self.out_topic}")
        rospy.loginfo(f"[NounReadingPublisher] word_file={self.word_file}")

    def speech_callback(self, msg):
        if not hasattr(msg, "transcript") or not msg.transcript:
            return

        spoken_word = (msg.transcript[0] or "").strip()
        if not spoken_word:
            return

        ok, hira = self._noun_only_hiragana_reading(spoken_word)
        if not ok or not hira:
            return

        # ★ ここで毎回ファイルを読み直す（起動後の更新を反映）
        target_initial, raw_word = self._load_last_char_with_debug()

        # target_initial が None のときは無条件で通す
        if target_initial is None or hira[0] == target_initial:
            self.pub.publish(String(data=spoken_word))
            rospy.loginfo(
                f"Published: spoken_word='{spoken_word}' reading='{hira}' "
                f"target='{target_initial}' from_file_word='{raw_word}'"
            )
        else:
            rospy.loginfo(
                f"Blocked: spoken_word='{spoken_word}' reading='{hira}' "
                f"target='{target_initial}' from_file_word='{raw_word}'"
            )

    def _load_last_char_with_debug(self):
        """
        Returns (target_initial or None, raw_word)
        raw_word: ファイルの中身（strip後）をログ用に返す
        """
        try:
            with open(self.word_file, "r", encoding="utf-8") as f:
                word = f.read()
            # 改行や空白を除去
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
