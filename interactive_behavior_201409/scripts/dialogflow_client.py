#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
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
from sound_play.msg import SoundRequestAction, SoundRequestGoal


class DialogflowClient(object):
    [IDLE,
     SPEAKING,
     LISTENING,
     THINKING,] = range(4)

    def __init__(self):
        self.project_id = rospy.get_param("~project_id")

        self.language = rospy.get_param("~language", "ja-JP")
        self.use_audio = rospy.get_param("~use_audio", False)
        self.use_speech = rospy.get_param("~use_speech", False)
        self.hotword = rospy.get_param("~hotword", "ねえねえ")

        self.state = self.IDLE
        self.session_id = None
        self.session_client = df.SessionsClient()
        self.queue = Queue.Queue()

        if self.use_speech:
            self.sound_action = actionlib.SimpleActionClient(
                "robotsound_jp", SoundRequestAction)
            if not self.sound_action.wait_for_server(rospy.Duration(5.0)):
                self.sound_action = None
        else:
            self.sound_action = None

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
        if msg.data == self.hotword:
            rospy.loginfo("Hotword received")
            self.state = self.LISTENING

    def input_cb(self, msg):
        if not self.use_audio:
            self.hotword_cb(String(data=msg.transcript[0]))
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

    def speak_result(self, result):
        if self.sound_action is None:
            return
        goal = SoundRequestGoal(
            command=SoundRequest.PLAY_ONCE,
            sound=SoundRequest.SAY,
            volume=1.0,
            arg=result.response,
            arg2=self.language)
        self.sound_action.send_goal_and_wait(goal, rospy.Duration(10.0))

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
                    result = self.detect_intent_text(msg.transcript[0], session)
                else:
                    raise RuntimeError("Invalid data")
                self.print_result(result)
                self.publish_result(result)
                self.speak_result(result)
                # TODO: if end of speech, return state to idle
            except Queue.Empty:
                pass
            except Exception as e:
                rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node("dialogflow_client")
    dfc = DialogflowClient()
    rospy.spin()
