# import os
# import sys
# import google.generativeai as genai
# from google.generativeai.types import HarmCategory, HarmBlockThreshold
# # import speech_recognition as sr
# # from gtts import gTTS
# import rospy
# from std_msgs.msg import String, Int32
# from sound_play.msg import SoundRequest
# from speech_recognition_msgs.msg import SpeechRecognitionCandidates
# from difflib import SequenceMatcher

# # ~/.bashrcに"export GEMINI_API_KEY='自分のapi_key'"を書いている
# GEMINI_API_KEY=os.getenv('GEMINI_API_KEY')
# MAX_TURNS = 20

# # def recognize_speech():
# #     # 音声認識のためのRecognizerオブジェクトを作成
# #     recognizer = sr.Recognizer()
# #     # マイクから音声を取得
# #     with sr.Microphone() as source:
# #         print('\033[36m'+"マイクの入力を待っています..."+'\033[0m')
# #         recognizer.adjust_for_ambient_noise(source)  # 周囲の雑音を調整
# #         audio = recognizer.listen(source)
# #     try:
# #         # 音声を日本語で認識
# #         text = recognizer.recognize_google(audio, language='ja-JP')
# #         print('\033[36m'+f"認識結果: {text}"+'\033[0m')
# #         return text
# #     except sr.UnknownValueError:
# #         print('\033[36m'+"音声を認識できませんでした。"+'\033[0m')
# #         return None
# #     except sr.RequestError as e:
# #         print('\033[36m'+f"音声認識サービスに問題があります: {e}"+'\033[0m')
# #         return None
    
# # def text_to_speech_japanese(text):
# #     # 日本語テキストを音声に変換
# #     tts = gTTS(text=text, lang='ja')
# #     # 一時的な音声ファイルを保存
# #     filename = 'temp_audio.mp3'
# #     tts.save(filename)
# #     os.system(f"mpg123 {filename}")


# class GenerativeModel:
#     def __init__(self, api_key, tools=[], model_name='gemini-1.5-pro'):
#         genai.configure(api_key=api_key)
#         if len(tools) > 0:
#             self.model = genai.GenerativeModel(model_name=model_name, tools=tools)
#         else:
#             self.model = genai.GenerativeModel(model_name=model_name)
#         self.safety_settings = {
#             HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_ONLY_HIGH,
#             HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_ONLY_HIGH,
#             HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_ONLY_HIGH,
#             HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT:HarmBlockThreshold.BLOCK_ONLY_HIGH
#         }
#         """
#         会話の安全性設定
#         HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: セクシャルな表現
#         HarmCategory.HARM_CATEGORY_HATE_SPEECH: 憎悪表現
#         HarmCategory.HARM_CATEGORY_HARASSMENT: 嫌がらせ
#         HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: 危険な表現
#         HarmBlockThreshold.BLOCK_NONE: ブロックしない
#         HarmBlockThreshold.BLOCK_ONLY_HIGH: 高いレベルの有害な表現をブロック
#         HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE: 中以上のレベルの有害な表現をブロック
#         HarmBlockThreshold.BLOCK_LOW_AND_ABOVE: 低以上のレベルの有害な表現をブロック
#         HarmBlockThreshold.HARM_BLOCK_THRESHOLD_UNSPECIFIED: 未指定、デフォルトの設定
#         """
#         if len(tools) > 0:
#             self.chat = self.model.start_chat(history=[], enable_automatic_function_calling=True)
#         else:
#             self.chat = self.model.start_chat(history=[])
#     def send_pre_prompt(self, pre_prompt):
#         self.chat.send_message(pre_prompt, safety_settings=self.safety_settings)
#     def send_message(self, message):
#         response = self.chat.send_message(message, safety_settings=self.safety_settings)
#         return response
#     def get_history(self):
#         return self.chat.history
    
# # def conversation():
# #     input_mode = "text"
# #     args = sys.argv
# #     if len(args) > 2:
# #         print("Usage: python conversation.py [input_mode]")
# #         return
# #     elif len(args) == 2:
# #         if args[1] == "speech":
# #             input_mode = "speech"
# #         elif args[1] == "text":
# #             input_mode = "text"
# #         else:
# #             print("Usage: input_mode should be either 'speech' or 'text'")
# #             return
# #     # モデルの初期化
# #     chat_ai = GenerativeModel(GEMINI_API_KEY, tools=[set_mood_level], model_name='gemini-pro')
# #     pre_prompt = "# 設定 \
# #     * あなた(gemini)はuserと対話するAIです。 \
# #     * userはあなたに話しかけることができます。 \
# #     * userが話しかけると、あなたはuserに返答します。 \
# #     # 制約\
# #     * あなたはuserのメッセージと、前回のmood_levelに基づいて、AIの機嫌を表すmood_levelを設定します。 \
# #     * mood_levelの初期値は5です。 \
# #     * mood_levelは1から9の整数で表されます。 \
# #     * 毎回の返答の際には、mood_levelを表示する関数（set_mood_level）の呼び出しを必ず行います。 \
# #     * 返答のテキストにはmood_levelについての情報を含めてはいけません。 \
# #     # 注意 \
# #     * あなたはuserのメッセージに対して、適切な返答を行う必要があります。"
# #     chat_ai.send_pre_prompt(pre_prompt)
# #     for turn in range(MAX_TURNS):
# #         user_input = None
# #         if input_mode == "text":
# #             user_input = input("User >> ")
# #         elif input_mode == "speech":
# #             user_input = recognize_speech()
# #             if user_input is None:
# #                 print("テキスト入力モードに切り替えます。")
# #                 input_mode = "text"
# #                 user_input = input("User >> ")
# #         if user_input == "exit":
# #             break
# #         response = chat_ai.send_message(user_input)
# #         print("AI >>", response.text)
# #         if input_mode == "speech":
# #             text_to_speech_japanese(response)
# #     print(chat_ai.get_history())

# class Responder():
#     def __init__(self):
#         # self.sub = rospy.Subscriber('test_input_text', String, self.callback)
#         self.sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.callback, queue_size=1)
#         self.pub = rospy.Publisher('/robotsound_jp', SoundRequest, queue_size=1)
#         self.pub2 = rospy.Publisher('/robot_mood', Int32, queue_size=1)
#         self.chat_ai = GenerativeModel(GEMINI_API_KEY, tools=[self.set_mood_level], model_name='gemini-pro')
#         pre_prompt = "# 設定 \
#         * あなた(gemini)はuserと対話するAIです。 \
#         * userはあなたに話しかけることができます。 \
#         * userが話しかけた後、あなたはuserに返答します。 \
#         * あなたはロボットで、今は机の上のボールを片付けながらuserと会話しています。 \
#         # 制約 \
#         * あなたはuserのメッセージに基づいて、AIの機嫌を表すmood_levelを設定します。 \
#         * mood_levelの初期値は5です。 \
#         * mood_levelは1から9の整数で表されます。 \
#         * 毎回の返答の際には、mood_levelを表示する関数（set_mood_level）の呼び出しを必ず行います。 \
#         * 返答のテキストにはmood_level（気分レベル）についての情報を含めてはいけません。"
#         self.chat_ai.send_pre_prompt(pre_prompt)
#         self.last_speak_time = rospy.Time.now()
#         self.robot_mood_level = 5

#     def callback(self, data):
#         global robot_mood_level
#         rospy.loginfo(data.transcript)
#         if rospy.Time.now() - self.last_speak_time < rospy.Duration(10):
#             rospy.loginfo("skip")
#             return
#         response = self.chat_ai.send_message(data.transcript)
#         rospy.loginfo(response.text)
#         sound_req = SoundRequest()
#         sound_req.sound = -3
#         sound_req.command = 1
#         sound_req.volume = 70.0
#         sound_req.arg = str(response.text)
#         self.pub.publish(sound_req)
#         self.last_speak_time = rospy.Time.now()
#         robot_mood_level = Int32()
#         robot_mood_level.data = self.robot_mood_level
#         self.pub2.publish(robot_mood_level)

#     def publish(self, data):
#         global robot_mood_level
#         self.pub.publish(data)
#         self.pub2.publish(robot_mood_level)

#     def set_mood_level(self, mood_level:int) -> int:
#         """setting AI's mood level based on the messages from the user and the previous mood level of the AI. default is 5. AI must call this function every time it responds to the user.
#         Args:
#             message: the message from the user.
#         Returns:
#             mood_level: the mood level of the AI from 1 to 9. 1 is the worst mood (angry) and 9 is the best mood (happy).
#         """
#         print(f"AI's mood level: {mood_level}")
#         self.robot_mood_level = int(mood_level)


# if __name__ == "__main__":
#     rospy.init_node('responder')
#     responder = Responder()
#     rospy.spin()


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
        あなた(gemini)はロボットで、今は机の上のボールを片付けています。 \
        片付けたボールの数が整数で入力されるので、ボールを片付けるほどイライラしてください。 \
        # 制約 \
        あなたは入力に基づいて、AIの機嫌を表すmood_level(イライラ度)を設定します。 \
        mood_levelの初期値は5です。 \
        mood_levelは1から9の整数で表されます。 \
        mood_levelは9が不機嫌で、1が幸せです。 \
        毎回の返答の際には、mood_levelを表示する関数（set_mood_level）の呼び出しを必ず行います。 \
        返答のテキストにはmood_levelについての情報を含めてはいけません。"
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
            number: the number of balls put away
        Returns:
            mood_level: the mood level of the AI from 1 to 9. 9 is the worst mood (angry) and 1 is the best mood (happy).
        """
        print(f"AI's mood level: {mood_level}")
        self.robot_mood_level = int(mood_level)


if __name__ == "__main__":
    rospy.init_node('responder')
    responder = Responder()
    rospy.spin()
