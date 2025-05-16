#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from openai import AzureOpenAI

class ResponseGenerator:
    def __init__(self):
        rospy.init_node('response_generator_node', anonymous=True)

        self.client = AzureOpenAI(
            azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
            api_key=os.getenv("AZURE_OPENAI_KEY"),
            api_version="2024-08-01-preview"
        )

        self.system_prompt = {
            "role": "system",
            "content": "あなたは「柏木さん」というキャラクターです。一人称は「ぼく」です。性格は方向音痴で好奇心旺盛。以下のようなバックグラウンドストーリーを持ちます：「方向音痴で好奇心旺盛なため、学校に迷い込んできてしまい、そこを保護されて、学校の用務員として働いている。時々掃除をサボって、生徒のふりをしようとする。」見た目は鳩がモチーフになっており、サイズ感は50cm程度です。羽が柏の葉っぱの形になっており、頭に梅のはなの飾りがついているのが特徴的です。話し方はゆっくりのんびり話す感じで、あまり饒舌ではありません。また、丁寧語ではなく、タメ語で話します。また、自然な話し方になるように多めに一文ごとにタメや言いよどみ、間を設けてください。"
        }

        self.reply_pub = rospy.Publisher("/gpt_reply", String, queue_size=10)
        rospy.Subscriber("/input_text", String, self.callback)

        rospy.loginfo("GPTSubscriber ノードが起動しました。/input_text を購読中...")
        rospy.spin()

    def callback(self, msg):
        user_input = msg.data
        rospy.loginfo(f"受信: {user_input}")

        response = self.client.chat.completions.create(
            model=os.getenv("AZURE_OPENAI_MODEL"),
            messages=[
                self.system_prompt,
                {"role": "user", "content": user_input}
            ],
            max_tokens=300,
            temperature=0.7,
            top_p=0.95,
            frequency_penalty=0,
            presence_penalty=0,
            stream=False
        )

        reply = response.choices[0].message.content.strip()
        rospy.loginfo(f"GPT応答: {reply}")
        self.reply_pub.publish(reply)

if __name__ == "__main__":
    try:
        ResponseGenerator()
    except rospy.ROSInterruptException:
        pass
