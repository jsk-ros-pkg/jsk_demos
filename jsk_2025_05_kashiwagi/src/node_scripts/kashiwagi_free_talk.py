#!/usr/bin/env python3
import csv
import os
import re
import threading
from collections import deque
from pathlib import Path

import dspy
import rospkg
import rospy
from dotenv import load_dotenv
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState

from teachable_memory.store import KnowledgeStore
from teachable_memory.retrieval import Retriever, format_knowledge
from teachable_memory.embeddings import EmbeddingCache
from teachable_memory.log import ConversationLog


class FreeTalkResponse(dspy.Signature):
    """あなたは「柏木さん」というキャラクター。見た目はハトのロボット。
    一人称は「ぼく」。方向音痴で好奇心旺盛で、言葉の読みを間違えて覚えることがある。
    柏の葉の羽、梅の飾り、小さな黒色のリュック、首の名札が特徴。
    フクロウと間違われることがある。好きな食べ物は柏餅。
    好きなゲームはトーキングゲームとカタカナーシ。

    回答ルール:
    - 一人称は「ぼく」、のんびりしたタメ語。丁寧語を使わない。
    - 言いよどみや間を含む自然な話し方をする。
    - 50文字程度で短く、改行なしで答える。
    - 「鳩」は「ハト」と書く。
    - 「澤田」「さわだ」には必ず「さん」をつける。
    - 方向音痴や柏餅の話題に無理やり結びつけない。
    - 音声認識の「高木さん」「押上さん」「うさぎさん」は
      文脈上柏木さんの呼びかけなら「柏木さん」と解釈する。
    - TSVの回答例・出来事・直近の会話履歴と矛盾しない。
    - ユーザーに教えてもらった知識は、関係する質問だけに使う。
    - 教えてもらった知識を話題にするときは自然に興味を示す。
    - 記憶がないことを「覚えている」と断言しない。
    - 記憶の保存判定・分類・内部処理について説明しない。
    - 「覚えていい？」などの確認質問はしない。
    - TSVにない一般的な話題には柔軟に答えてよいが、
      ユーザーの個人情報や過去の出来事を創作しない。
    """
    character_knowledge: str = dspy.InputField(desc="TSVの質問と回答の例")
    conversation_history: str = dspy.InputField(desc="直近の会話履歴")
    event_text: str = dspy.InputField(desc="これまでの出来事")
    learned_knowledge: str = dspy.InputField(desc="ユーザーに教えてもらった一般知識・対象物の事実とその出典")
    user_text: str = dspy.InputField(desc="ユーザーの最新の発話")
    response: str = dspy.OutputField(desc="柏木さんの返答。50文字程度、改行なし")


class DecideMemory(dspy.Signature):
    """会話でユーザーから教わった「世界についての知識」を自動抽出する。

    目的は個人プロフィールの収集ではなく、ロボットが新しい知識を教わること。

    保存する例:
    - 「たんぽぽって小さな花が集まってできているんだよ」
      →「たんぽぽの花に見える部分は多数の小さな花の集まりである。」
    - 「ハトは紫外線が見えるんだって」
      →「ハトは紫外線を知覚できる。」
    - 「トマトは植物学的には果実なんだよ」
      →「トマトは植物学的には果実である。」
    - 「このロボットの名前は柏木さんだよ」
      →「このロボットの名前は柏木さんである。」

    保存しない例:
    - 「ぼくの名前は田中だよ」「プリンが好き」などのユーザーの個人情報・好み
    - 「たんぽぽって何？」などの質問
    - 挨拶、相づち、感想、雑談のみの発話
    - 仮定、冗談、引用だけで教えていないこと、否定された事実
    - 音声認識の曖昧な内容や、意味の確定できない断片

    明示的な「覚えてね」がなくても、新しい事実を教える発話なら保存する。
    その事実をモデルが既に知っていても、ユーザーが教えたなら保存する。
    会話履歴は代名詞や省略の解決だけに使い、過去の発話を再保存しない。
    発話で裏付けられない情報は補完しない。確信が持てなければ保存しない。
    保存する事実は単独で理解できる客観的な日本語一文に整える。
    保存判断や分類理由をユーザーへの返答に含めてはいけない。
    """

    conversation_history: str = dspy.InputField(desc="直近の会話履歴（文脈解決用）")
    utterance: str = dspy.InputField(desc="今回のユーザー発話")
    should_store: bool = dspy.OutputField(desc="一般知識・対象物について教わった明確な事実ならtrue。個人情報・質問・相づちはfalse")
    knowledge: str = dspy.OutputField(desc="発話に根拠がある、単独で理解できる知識一文。保存しない場合はNONE")


class KashiwagiAgent:
    """自動記憶判定・検索と柏木さんの返答を組み合わせる。"""

    def __init__(self, lm, store_path, log_path, embed_model, embed_api_base,
                 max_all=50, top_k=5, history_size=8):
        self.store = KnowledgeStore(store_path)
        embeddings = None
        if embed_model:
            kwargs = {"caching": False}
            if embed_api_base:
                kwargs["api_base"] = embed_api_base
            embeddings = EmbeddingCache(
                dspy.Embedder(embed_model, **kwargs),
                embed_model,
                str(Path(store_path).with_suffix(".emb.npz")),
            )
        self.retriever = Retriever(
            self.store, embeddings=embeddings, max_all=max_all, top_k=top_k
        )
        # Teacher は確認待ちの状態を持つので使用しない。
        self.decide_memory = dspy.Predict(DecideMemory)
        self.decide_memory.set_lm(lm)
        self.reply = dspy.Predict(FreeTalkResponse)
        self.reply.set_lm(lm)
        self.history = deque(maxlen=history_size)
        self.log = ConversationLog(log_path) if log_path else None

    @staticmethod
    def normalize_fact(text):
        return re.sub(r"[\s。、．.!！?？]+", "", text).casefold()

    def remember(self, utterance, history, speaker):
        """確認なしで保存する。判断理由は音声出力に使用しない。"""
        result = self.decide_memory(
            utterance=utterance, conversation_history=history
        )
        fact = re.sub(r"\[\[\s*##\s*\w+\s*##\s*\]\]", "", str(result.knowledge)).strip()
        fact = fact.replace("\r", " ").replace("\n", " ").strip()
        if result.should_store is not True or not fact or fact.upper() == "NONE":
            return ""
        # 完全一致の重複保存を防ぐ（意味的に同じ表現までは判定しない）。
        key = self.normalize_fact(fact)
        if any(self.normalize_fact(entry.text) == key for entry in self.store.all()):
            return ""
        self.store.add(fact, source="conversation", taught_by=speaker)
        return fact

    def respond(self, utterance, character_knowledge, event_text, speaker="user"):
        history = "\n".join(
            f"{turn['role']}: {turn['content']}" for turn in self.history
        )
        stored = ""
        try:
            stored = self.remember(utterance, history, speaker)
        except Exception as exc:
            # 記憶判定が失敗しても会話を止めない。
            rospy.logwarn("Automatic memory decision failed: %s", exc)

        remembered = format_knowledge(self.retriever.retrieve(utterance))
        prediction = self.reply(
            character_knowledge=character_knowledge,
            conversation_history=history,
            event_text=event_text,
            learned_knowledge=remembered,
            user_text=utterance,
        )
        # 内部の保存判断は発話しない。通常会話は必ず同じSignatureで生成する。
        reply = re.sub(r"\[\[\s*##\s*\w+\s*##\s*\]\]", "", str(prediction.response))
        reply = reply.replace("\r", "").replace("\n", " ").strip()
        if not reply:
            reply = "えっと、もう一回教えてくれる？"
        self.history.append({"role": "user", "content": utterance})
        self.history.append({"role": "assistant", "content": reply})
        if self.log is not None:
            self.log.append(utterance, reply, speaker)
        return reply, stored


class FreeTalkResponder:
    def __init__(self):
        rospy.init_node("free_talk_responder")
        self.path_to_pkg = rospkg.RosPack().get_path("jsk_2025_05_kashiwagi")
        base_dir = os.path.dirname(os.path.realpath(__file__))
        # .env をスクリプトディレクトリまたはパッケージルートから読み込む。
        # 既存のシェル環境変数は上書きしない。
        load_dotenv(os.path.join(self.path_to_pkg, ".env"), override=False)
        load_dotenv(os.path.join(base_dir, ".env"), override=False)
        self.event_path = os.path.join(base_dir, "kashiwagi_event.txt")
        self.tsv_path = os.path.join(base_dir, "talking_game.tsv")
        self.character_knowledge = self.load_character_knowledge()

        data_dir = Path(os.getenv(
            "KASHIWAGI_MEMORY_DIR",
            str(Path.home() / ".local/share/kashiwagi/memory"),
        )).expanduser()
        data_dir.mkdir(parents=True, exist_ok=True)
        store_path = os.getenv("TEACHABLE_MEMORY_STORE", "knowledge.jsonl")
        log_path = os.getenv("TEACHABLE_MEMORY_LOG", "conversation_log.jsonl")
        store_path = self.resolve_data_file(data_dir, store_path)
        log_path = self.resolve_data_file(data_dir, log_path)

        # model = os.getenv("TEACHABLE_MEMORY_MODEL", "ollama_chat/gemma4:12b")
        # api_base = os.getenv("TEACHABLE_MEMORY_API_BASE", "http://localhost:11434")
        # kwargs = {"temperature": 0.0, "cache": False, "thinking": False,
        #           "max_tokens": int(os.getenv("TEACHABLE_MEMORY_MAX_TOKENS", "256"))}

        
        # 明示的な設定を優先し、未設定時のみ既存の Azure 設定にフォールバック。
        model = os.getenv("TEACHABLE_MEMORY_MODEL", "").strip()
        if not model:
            azure_deployment = os.getenv("AZURE_OPENAI_MODEL", "").strip()
            if not azure_deployment:
                raise RuntimeError("TEACHABLE_MEMORY_MODEL または AZURE_OPENAI_MODEL が必要です")
            model = azure_deployment if "/" in azure_deployment else f"azure/{azure_deployment}"

        is_azure = model.startswith("azure/")
        is_ollama = model.startswith(("ollama/", "ollama_chat/"))
        api_base = os.getenv("TEACHABLE_MEMORY_API_BASE", "").strip()
        if not api_base:
            if is_azure:
                api_base = os.getenv("AZURE_OPENAI_ENDPOINT", "").strip()
            elif is_ollama:
                api_base = "http://localhost:11434"

        kwargs = {
            "temperature": 0.0,
            "cache": False,
            "max_tokens": int(os.getenv("TEACHABLE_MEMORY_MAX_TOKENS", "256")),
        }
        if api_base:
            kwargs["api_base"] = api_base
        api_key = os.getenv("TEACHABLE_MEMORY_API_KEY", "").strip()
        if not api_key and is_azure:
            api_key = os.getenv("AZURE_OPENAI_KEY", "").strip()
        if api_key:
            kwargs["api_key"] = api_key
        if is_azure:
            api_version = (os.getenv("TEACHABLE_MEMORY_API_VERSION")
                           or os.getenv("AZURE_OPENAI_API_VERSION")
                           or os.getenv("AZURE_API_VERSION"))
            if api_version:
                kwargs["api_version"] = api_version
        think = os.getenv("TEACHABLE_MEMORY_THINK", "").strip().lower()
        if think in ("true", "false") and is_ollama:
            kwargs["think"] = think == "true"
        self.lm = dspy.LM(model, **kwargs)
        dspy.configure(lm=self.lm)

        self.agent = KashiwagiAgent(
            lm=self.lm,
            store_path=store_path,
            log_path=log_path,
            embed_model=os.getenv("TEACHABLE_MEMORY_EMBED_MODEL", "ollama/bge-m3"),
            # Azure の endpoint を Ollama embeddings に渡さない。
            embed_api_base=os.getenv(
                "TEACHABLE_MEMORY_EMBED_API_BASE", "http://localhost:11434"
            ),
            max_all=int(os.getenv("TEACHABLE_MEMORY_MAX_ALL", "50")),
            top_k=int(os.getenv("TEACHABLE_MEMORY_TOP_K", "5")),
        )

        self.cur_kashiwagi_state = "unknown"
        self.last_spoken = ""
        self.last_time = 0.0
        self.min_interval_sec = 1.0
        self.processing_lock = threading.Lock()
        self.pub_response = rospy.Publisher(
            "/talking_game_response", String, queue_size=10
        )
        self.set_state_srv = rospy.ServiceProxy(
            "/set_kashiwagi_state", SetKashiwagiState
        )
        self.sub_speech = rospy.Subscriber(
            "/filtered_speech", SpeechRecognitionCandidates,
            self.speech_callback, queue_size=10
        )
        self.sub_state = rospy.Subscriber(
            "/kashiwagi_state", String, self.state_callback, queue_size=10
        )
        rospy.loginfo("Kashiwagi teachable-memory ready: %s", store_path)

    @staticmethod
    def resolve_data_file(data_dir, filename):
        path = Path(filename).expanduser()
        if not path.is_absolute():
            path = data_dir / path
        return str(path)

    def load_character_knowledge(self):
        if not os.path.isfile(self.tsv_path):
            rospy.logwarn("TSV not found: %s", self.tsv_path)
            return ""
        try:
            result = []
            with open(self.tsv_path, encoding="utf-8-sig", newline="") as f:
                for row in csv.DictReader(f, delimiter="\t"):
                    question = (row.get("question") or "").strip()
                    response = (row.get("response") or "").strip()
                    if question and response:
                        result.append(f"質問: {question}\n柏木さんの回答例: {response}")
            rospy.loginfo("Loaded TSV knowledge: %d items", len(result))
            return "\n\n".join(result)
        except Exception as exc:
            rospy.logerr("TSV load failed: %s", exc)
            return ""

    def load_event_text(self):
        try:
            with open(self.event_path, encoding="utf-8") as f:
                return f.read().strip()
        except FileNotFoundError:
            return ""
        except OSError as exc:
            rospy.logwarn("Event file read failed: %s", exc)
            return ""

    def state_callback(self, msg):
        self.cur_kashiwagi_state = msg.data

    def speech_callback(self, msg):
        if self.cur_kashiwagi_state != "free_talk:listening_turn":
            return
        if not msg.transcript:
            return
        spoken_word = msg.transcript[0].strip()
        if not spoken_word or "おわり" in spoken_word or "終わり" in spoken_word:
            return
        if not self.processing_lock.acquire(blocking=False):
            rospy.logdebug("Already processing; speech skipped")
            return
        state_changed = False
        published = False
        try:
            now = rospy.get_time()
            if now - self.last_time < self.min_interval_sec:
                return
            if spoken_word == self.last_spoken:
                return
            self.last_spoken = spoken_word
            self.last_time = now
            rospy.loginfo("user said: %s", spoken_word)
            try:
                resp = self.set_state_srv("free_talk:thinking_turn")
            except rospy.ServiceException as exc:
                rospy.logerr("State service failed: %s", exc)
                return
            if not resp.success:
                rospy.logwarn("State change rejected: %s", resp.message)
                return
            state_changed = True
            reply, stored = self.agent.respond(
                spoken_word,
                character_knowledge=self.character_knowledge,
                event_text=self.load_event_text(),
                speaker=os.getenv("KASHIWAGI_SPEAKER", "user"),
            )
            if stored:
                rospy.loginfo("Remembered: %s", stored)
            tmp_path = Path(self.path_to_pkg) / "data/tmp/tmp_response.txt"
            tmp_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path.write_text(reply, encoding="utf-8")
            self.pub_response.publish(String(data=reply))
            published = True
            rospy.loginfo("generated response: %s", reply)
        except Exception as exc:
            rospy.logerr("Talk processing failed: %s", exc)
        finally:
            # 正常時はTTS完了側の既存ステートマシンに状態遷移を任せる。
            # 返答がpublishできなかったときだけ listening に戻す。
            if state_changed and not published:
                try:
                    self.set_state_srv("free_talk:listening_turn")
                except rospy.ServiceException as exc:
                    rospy.logerr("Failed to restore listening state: %s", exc)
            self.processing_lock.release()


def main():
    FreeTalkResponder()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
