#!/usr/bin/env python3

import rospy
from collections import deque
from difflib import SequenceMatcher

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_to_text_corrector import DSPyCorrector


class CorrectedSpeechToTextPublisher:
    def __init__(self):
        rospy.init_node("corrected_speech_to_text_publisher")
        self.corrector = DSPyCorrector()
        self.history=deque(maxlen=6)
        self.pub = rospy.Publisher("/corrected_speech_to_text", SpeechRecognitionCandidates, queue_size=10)
        self.sub = rospy.Subscriber("/speech_to_text/raw", SpeechRecognitionCandidates, self.callback, queue_size=10)
        
        self.min_wait_time_for_next_asr_result = 0.8
        self.max_wait_time_for_next_asr_result = 2.0
        self.pending_msg = None
        self.timer = None
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.last_processed_text = None
        self.first_result_time = None
        rospy.loginfo("corrected_speech_to_text_publisher started")

    def callback(self, raw_msg):
        if not raw_msg.transcript:
            return

        raw_text = raw_msg.transcript[0].strip()

        if not raw_text:
            return
        rospy.loginfo(f"Result candidate: {raw_text}")
        now = rospy.Time.now()
        
        if self.pending_msg is None:
            self.first_result_time = now
            
        self.pending_msg = raw_msg
        self.last_result_time = now

    def timer_callback(self, event):
        if self.pending_msg is None:
            return
        now = rospy.Time.now()
        # 最後にメッセージが来てからmin_wait_time_for_next_asr_result以上経過した場合はcorrectorで処理する
        # ↑こまめに処理しすぎないための時間間隔指定
        time_from_last_result_msg = (now - self.last_result_time).to_sec()
        
        # 最初のresultからmax_wait_time_for_next_asr_result以上経過した場合はcorrectorで処理する
        # ↑ずっとreusltが送られ続ける(e.g., 騒音環境など)場合も対応するための時間間隔の指定
        time_from_first_result_msg  = (now - self.first_result_time).to_sec()

        # 最後の処理からmin_wait_time_for_next_asr_result秒以内 かつ 最初の処理からmax_wait_time_for_next_asr_result以内ならばcorrectorは走らせない
        if time_from_last_result_msg < self.min_wait_time_for_next_asr_result and time_from_first_result_msg < self.max_wait_time_for_next_asr_result:
            return
        raw_msg = self.pending_msg
        self.pending_msg = None
        self.first_result_time = None
        self.process_result(raw_msg)

    def judge_asr_result(self, previous, current):
        """
        前回処理したASR結果と今回のASR結果を比較する。

        Returns:
        "DROP"         : ほぼ同じなので無視
        "PROCESS_DIFF" : 前回の続きとみなし、新しい部分だけ処理
        "PROCESS_FULL" : 別発話として全文処理
        """

        previous = previous.strip() if previous else ""
        current = current.strip()

        if not previous:
            return "PROCESS_FULL", current

        if previous == current:
            return "DROP", ""

        similarity = SequenceMatcher(None, previous, current).ratio()
        length_diff = abs(len(current) - len(previous))

        if similarity >= 0.9 and length_diff <= 3:
            return "DROP", ""

        if similarity >= 0.75 and len(current) > len(previous):
            matcher = SequenceMatcher(None, previous, current)
            match = matcher.find_longest_match(0, len(previous), 0, len(current))
            overlap_ratio = match.size / len(previous)

            if overlap_ratio >= 0.7:
                new_part_start = match.b + match.size
                new_part = current[new_part_start:].strip(" 、。！？!?，．")
                if not new_part:
                    return "DROP", ""
                return "PROCESS_DIFF", new_part
        return "PROCESS_FULL", current
        
        
    def process_result(self, raw_msg):
        raw_text = raw_msg.transcript[0].strip()
        decision, text_to_process = self.judge_asr_result(self.last_processed_text, raw_text)

        rospy.loginfo(f"last_processed_text = {self.last_processed_text}")
        rospy.loginfo(f"decision = {decision}")
        rospy.loginfo(f"text_to_process = {text_to_process}")

        if decision == "DROP":
            rospy.loginfo("DROP: similar ASR result")
            return

        print(f"history = {self.history}")
        
        corrected_msg = SpeechRecognitionCandidates()
        #corrected_text = self.corrector.correct(text_to_process, self.history)
        corrected_text = text_to_process
        corrected_msg.transcript = [corrected_text]
        self.history.append(f"User: {corrected_text}")

        rospy.loginfo(f"decision = {decision}")
        rospy.loginfo(f"text_to_process = {text_to_process}")
        rospy.loginfo(f"***********************corrected_text  = {corrected_text}*********************")

        self.last_processed_text = raw_text

        self.pub.publish(corrected_msg)

if __name__=="__main__":
    node = CorrectedSpeechToTextPublisher()
    rospy.spin()
