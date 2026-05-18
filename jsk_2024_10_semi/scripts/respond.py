#!/usr/bin/env python3
import os
import sys
import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold
# import speech_recognition as sr
# from gtts import gTTS
import rospy
from std_msgs.msg import String, Int32
from sound_play.msg import SoundRequest
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
# from difflib import SequenceMatcher

# ~/.bashrcに"export GEMINI_API_KEY='自分のapi_key'"を書いている
GEMINI_API_KEY=os.getenv('GEMINI_API_KEY')
MAX_TURNS = 20

class GenerativeModel:
    def __init__(self, api_key, tools=[], model_name='gemini-1.5-pro'):
        genai.configure(api_key=api_key)
        if len(tools) > 0:
            self.model = genai.GenerativeModel(model_name=model_name, tools=tools)
        else:
            self.model = genai.GenerativeModel(model_name=model_name)
        self.safety_settings = {
            HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_ONLY_HIGH,
            HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_ONLY_HIGH,
            HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_ONLY_HIGH,
            HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT:HarmBlockThreshold.BLOCK_ONLY_HIGH
        }
        """
        会話の安全性設定
        HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: セクシャルな表現
        HarmCategory.HARM_CATEGORY_HATE_SPEECH: 憎悪表現
        HarmCategory.HARM_CATEGORY_HARASSMENT: 嫌がらせ
        HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: 危険な表現
        HarmBlockThreshold.BLOCK_NONE: ブロックしない
        HarmBlockThreshold.BLOCK_ONLY_HIGH: 高いレベルの有害な表現をブロック
        HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE: 中以上のレベルの有害な表現をブロック
        HarmBlockThreshold.BLOCK_LOW_AND_ABOVE: 低以上のレベルの有害な表現をブロック
        HarmBlockThreshold.HARM_BLOCK_THRESHOLD_UNSPECIFIED: 未指定、デフォルトの設定
        """
        if len(tools) > 0:
            self.chat = self.model.start_chat(history=[], enable_automatic_function_calling=True)
        else:
            self.chat = self.model.start_chat(history=[])
    def send_pre_prompt(self, pre_prompt):
        self.chat.send_message(pre_prompt, safety_settings=self.safety_settings)
    def send_message(self, message):
        response = self.chat.send_message(message, safety_settings=self.safety_settings)
        return response
    def get_history(self):
        return self.chat.history
    

class Responder():
    def __init__(self):
        # self.sub = rospy.Subscriber('test_input_text', String, self.callback)
        self.sub = rospy.Subscriber('ball_count', Int32, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/robotsound_jp', SoundRequest, queue_size=1)
        self.pub2 = rospy.Publisher('/robot_mood', Int32, queue_size=1)
        self.chat_ai = GenerativeModel(GEMINI_API_KEY, tools=[self.set_mood_level], model_name='gemini-pro')
        pre_prompt = "# 設定 \
        * あなた(gemini)はロボットで、今は机の上のボールを片付けています。 \
        * 片付けたボールの数が整数で入力されるので、片付けても片付けが進まないことに対してイライラしてください。 \
        # 制約 \
        * あなたは入力に基づいて、AIの機嫌を表すmood_levelを設定します。 \
        * mood_levelの初期値は5です。 \
        * mood_levelは1から9の整数で表されます。 \
        * 毎回の返答の際には、mood_levelを表示する関数（set_mood_level）の呼び出しを必ず行います。 \
        * 返答のテキストにはmood_level（気分レベル）についての情報を含めてはいけません。"
        self.chat_ai.send_pre_prompt(pre_prompt)
        self.last_speak_time = rospy.Time.now()
        self.robot_mood_level = 5

    def callback(self, data):
        global robot_mood_level
        rospy.loginfo(data.data)
        response = self.chat_ai.send_message(str(data.data))
        rospy.loginfo(response.text)
        sound_req = SoundRequest()
        sound_req.sound = -3
        sound_req.command = 1
        sound_req.volume = 70.0
        sound_req.arg = str(response.text)
        self.pub.publish(sound_req)
        self.last_speak_time = rospy.Time.now()
        robot_mood_level = Int32()
        robot_mood_level.data = self.robot_mood_level
        self.pub2.publish(robot_mood_level)

    def publish(self, data):
        global robot_mood_level
        self.pub.publish(data)
        self.pub2.publish(robot_mood_level)

    def set_mood_level(self, mood_level:int) -> int:
        """setting AI's mood level based on inputs from the user and the previous mood level of the AI. default is 5. AI must call this function every time it responds to the user.
        Args:
            message: the message from the user.
        Returns:
            mood_level: the mood level of the AI from 1 to 9. 1 is the worst mood (angry) and 9 is the best mood (happy).
        """
        print(f"AI's mood level: {mood_level}")
        self.robot_mood_level = int(mood_level)


if __name__ == "__main__":
    rospy.init_node('responder')
    responder = Responder()
    rospy.spin()
