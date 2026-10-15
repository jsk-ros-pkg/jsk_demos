#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String, Float32
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class QRToSpeechPublisher:
    def __init__(self):
        rospy.init_node("qr_to_speech_publisher")
        rospy.sleep(1)

        # 距離判定は参考プログラムに合わせる
        self.last_qr_distance = float("nan")
        self.qr_distance_threshold = 0.10

        # 同じQRを短時間に何度も処理しない
        self.recent_ids = {}
        self.cooldown_sec = 60

        # publish対象にするQRデータ
        self.allowed_qr_texts = {
            "遊ぼう",
            "歌",
            "おわり",
        }

        self.pub_speech = rospy.Publisher(
            "/speech_to_text",
            SpeechRecognitionCandidates,
            queue_size=10
        )

        rospy.Subscriber(
            "/qr_distance",
            Float32,
            self.depth_update_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/qr_data",
            String,
            self.qr_callback,
            queue_size=1
        )

        rospy.loginfo("QR to Speech Publisher started.")
        rospy.spin()

    def depth_update_callback(self, msg):
        self.last_qr_distance = msg.data

    def qr_callback(self, msg):
        qr_text = msg.data.strip()
        rospy.loginfo(f"QR data: {qr_text}")

        # 距離判定
        # 参考プログラムと同じく、thresholdより遠いと処理しない
        if self.last_qr_distance > self.qr_distance_threshold:
            rospy.logwarn("QR code is far from robot")
            return

        # 指定されたQRデータ以外は無視
        if qr_text not in self.allowed_qr_texts:
            rospy.logwarn(f"QR code data is not target text: {qr_text}")
            return

        # クールダウン判定
        now = time.time()
        if qr_text in self.recent_ids:
            elapsed_time = now - self.recent_ids[qr_text]
            if elapsed_time < self.cooldown_sec:
                rospy.loginfo(
                    f"this qr code is skipped because scanned {elapsed_time:.1f} sec ago"
                )
                return

        self.recent_ids[qr_text] = now

        # /speech_to_text に SpeechRecognitionCandidates として publish
        speech_msg = SpeechRecognitionCandidates()
        speech_msg.transcript = [qr_text]
        speech_msg.confidence = [1.0]

        self.pub_speech.publish(speech_msg)
        rospy.loginfo(f"published to /speech_to_text: {qr_text}")


if __name__ == "__main__":
    try:
        QRToSpeechPublisher()
    except rospy.ROSInterruptException:
        pass
