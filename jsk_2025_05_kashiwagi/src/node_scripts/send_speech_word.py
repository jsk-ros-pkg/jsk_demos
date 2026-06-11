#!/usr/bin/env python3
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class KeyboardToSpeechPublisher:
    def __init__(self):
        rospy.init_node("keyboard_to_speech_publisher")

        self.pub_speech = rospy.Publisher(
            "/speech_to_text",
            SpeechRecognitionCandidates,
            queue_size=10
        )

        rospy.loginfo("Keyboard to Speech Publisher started.")
        rospy.loginfo("Type any word and press Enter.")
        rospy.loginfo("Ctrl+C to exit.")

        self.keyboard_input_loop()

    def keyboard_input_loop(self):
        while not rospy.is_shutdown():
            try:
                text = input("> ").strip()

                if text == "":
                    continue

                self.publish_speech(text)

            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("Exiting keyboard_to_speech_publisher.")
                break

    def publish_speech(self, text):
        msg = SpeechRecognitionCandidates()
        msg.transcript = [text]
        msg.confidence = [1.0]

        self.pub_speech.publish(msg)
        rospy.loginfo(f"published to /speech_to_text: {text}")


if __name__ == "__main__":
    try:
        KeyboardToSpeechPublisher()
    except rospy.ROSInterruptException:
        pass
