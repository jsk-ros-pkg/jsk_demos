#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import dialogflow as df
from google.protobuf.json_format import MessageToJson
from google.protobuf.text_format import MessageToString
import pprint
import Queue
import rospy
import threading
import uuid
import webrtcvad

from audio_common_msgs.msg import AudioData
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from interactive_behavior_201409.msg import DialogResponse


class DialogflowClient(object):
    [IDLE,
     SPEAKING,
     LISTENING,
     THINKING,] = range(4)

    def __init__(self):
        self.project_id = rospy.get_param("~project_id")

        self.language = rospy.get_param("~language", "ja-JP")
        self.use_audio = rospy.get_param("~use_audio", False)

        self.state = self.IDLE
        self.session_id = None
        self.session_client = df.SessionsClient()
        self.queue = Queue.Queue()

        self.pub_res = rospy.Publisher(
            "dialog_response", DialogResponse, queue_size=1)

        if self.use_audio:
            self.audio_sample_rate = rospy.get_param("~audio_sample_rate", 16000)
            self.audio_config = df.types.InputAudioConfig(
                audio_encoding=df.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16,
                language_code=self.language,
                sample_rate_hertz=self.audio_sample_rate)
            self.audio_data = None
            self.sub_hotword = rospy.Subscriber(
                "hotword", String, self.hotword_cb)
            self.sub_audio = rospy.Subscriber(
                "speech_audio", AudioData, self.input_cb)
        else:
            self.sub_speech = rospy.Subscriber(
                "speech_to_text", SpeechRecognitionCandidates,
                self.input_cb)

        self.df_thread = threading.Thread(target=self.df_run)
        self.df_thread.daemon = True
        self.df_thread.start()

    def hotword_cb(self, msg):
        rospy.loginfo("Hotword received")
        self.state = self.LISTENING

    def input_cb(self, msg):
        if self.state != self.IDLE:
            self.queue.put(msg)
            rospy.loginfo("Received input")

    def detect_intent_text(self, data, session):
        query = df.types.QueryInput(
            text=df.types.TextInput(
                text=data, language_code=self.language))
        return self.session_client.detect_intent(
            session=session, query_input=query).query_result

    def detect_intent_audio(self, data, session):
        query = df.types.QueryInput(audio_config=self.audio_config)
        return self.session_client.detect_intent(
            session=session, query_input=query,
            input_audio=data).query_result

    def print_result(self, result):
        rospy.loginfo(pprint.pformat(result))
        # rospy.loginfo('Query: %s' % result.query_result.query_text)
        # rospy.loginfo('Intent: %s (confidence: %f)' % (
        #     result.query_result.intent.display_name,
        #     result.query_result.intent_detection_confidence))
        # rospy.loginfo('Fulfillment text: %s' % result.query_result.fulfillment_text)

    def publish_result(self, result):
        msg = DialogResponse()
        msg.header.stamp = rospy.Time.now()
        if result.action is not 'input.unknown':
            rospy.logwarn("Unknown action")
        msg.query = result.query_text.encode("utf-8")
        msg.action = result.action
        msg.response = result.fulfillment_text.encode("utf-8")
        msg.fulfilled = result.all_required_params_present
        msg.parameters = MessageToJson(result.parameters)
        msg.speech_score = result.speech_recognition_confidence
        msg.intent_score = result.intent_detection_confidence
        self.pub_res.publish(msg)

    def df_run(self):
        while True:
            if rospy.is_shutdown():
                break
            try:
                msg = self.queue.get(timeout=0.1)
                rospy.loginfo("Processing")
                if self.session_id is None:
                    self.session_id = str(uuid.uuid1())
                    rospy.loginfo("Created new session: %s" % self.session_id)
                session = self.session_client.session_path(
                    self.project_id, self.session_id)

                if isinstance(msg, AudioData):
                    result = self.detect_intent_audio(msg.data, session)
                elif isinstance(msg, SpeechRecognitionCandidates):
                    result = self.detect_intent_string(msg.transcript[0], session)
                else:
                    raise RuntimeError("Invalid data")
                self.print_result(result)
                self.publish_result(result)
            except Queue.Empty:
                pass
            except Exception as e:
                rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node("dialogflow_client")
    dfc = DialogflowClient()
    rospy.spin()
