#!/usr/bin/env python3
import os
import dspy

class SpeechToTextCorrector:
    def rule_based_corrector(self, raw_text):
        replacement_examples_dict = {
            "こんにちは": "さようなら"
        }

        corrected_text = raw_text
        
        for wrong_example, corrected_example in replacement_examples_dict.items():
            corrected_text = corrected_text.replace(wrong_example, corrected_example)
        return corrected_text

class ASRCorrection(dspy.Signature):
    """
    会話履歴を参考にして，音声認識結果に含まれる
    明らかな認識誤りだけを修正する。

    ルール:
    - 会話履歴は誤認識を判断するためだけに使用する．
    - 発話者が話した内容をできるだけ保持する
    - 言い換えない
    - 要約しない
    - 主語や情報を追加しない
    - 固有名詞は明らかな場合だけ修正する
    - 不確かな箇所は変更しない
    - 元から正しい文章は変更しない
    - 明らかにノイズと考えられる認識内容は無視する
    """
    conversation_history: str = dspy.InputField(desc="現在の発話より前の直近の会話履歴")
    raw_text: str = dspy.InputField(desc="音声認識結果")
    corrected_text: str = dspy.OutputField(desc="明らかな音声認識誤りを修正した結果")

class DSPyCorrector:
    def __init__(self):
        # self.lm = dspy.LM(
        #     f"azure/gpt-5.6-terra",
        #     api_key=os.environ["AZURE_OPENAI_KEY"],
        #     api_base=os.environ["AZURE_OPENAI_ENDPOINT"],
        #     reasoning_effort="none",
        #     cache=False,
        #     )

        # self.lm = dspy.LM(
        #     f"ollama_chat/gpt-oss:20b",
        #     api_key="",
        #     api_base="http://localhost:11434",
        #     temperature=0,
        #     think="low",
        #     cache=False,
        #     )

        self.lm = dspy.LM(
            f"ollama_chat/gemma4:12b", 
            api_key="",
            api_base="http://localhost:11434",
            temperature=0,
            think=False,
            cache=False,
            )
        

        dspy.configure(lm=self.lm)
        self.predictor = dspy.Predict(ASRCorrection)

    def correct(self, text, conversation_history):
        conversation_history = "\n".join(conversation_history)
        result = self.predictor(conversation_history=conversation_history, raw_text=text)
        return result.corrected_text.strip()
            
