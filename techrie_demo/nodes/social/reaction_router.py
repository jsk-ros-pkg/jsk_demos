#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, yaml, time, json, os, sqlite3, random
from rospkg import RosPack
from std_msgs.msg import String
from threading import Lock, Timer


_speak_lock = ""  # 空文字ならロック無し

def _lock_cb(msg: String):
    global _speak_lock
    _speak_lock = (msg.data or "").strip()


SUPPORTED = [
    "praise", "pet", "offer_food", "show_art", "invite",
    "greeting_hello", "greeting_goodnight", "joke", "sorry"
]

def _read_profile(pkg_root):
    try:
        with open(os.path.join(pkg_root, "config", "Jedy.json"), "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {"name":"Jedy","likes":["orange","drawing"],"style":"friendly"}

def _read_recent_events(db_path, limit=10):
    rows = []
    try:
        con = sqlite3.connect(db_path)
        cur = con.cursor()
        cur.execute("SELECT ts,type,actor,target,meta_json FROM events ORDER BY ts DESC LIMIT ?", (limit,))
        for ts, et, actor, target, meta in cur.fetchall():
            try:
                m = json.loads(meta) if meta else {}
            except Exception:
                m = {}
            rows.append({"ts": ts, "event": et, "actor": actor, "target": target, "meta": m})
        con.close()
    except Exception:
        pass
    rows.reverse()
    return rows

def _pick_yaml_line(v):
    if not v:
        return ""
    if isinstance(v, str):
        return v
    if isinstance(v, list) and v:
        return random.choice(v)
    return ""

class GPTPhraseGen(object):
    """Azure/OpenAI (Azure優先) で台詞と感情を生成。設定は config/gpt_api.yaml から読む。"""
    def __init__(self, cfg_path):
        self._client_chat = None
        self._client_image = None
        self._chat = {}
        self._image = {}
        self._provider = "azure"

        if not os.path.exists(cfg_path):
            raise RuntimeError("gpt_api.yaml が見つかりません: %s" % cfg_path)

        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

        self._provider = (cfg.get("provider") or "azure").lower()
        self._chat = cfg.get("chat") or {}
        self._image = cfg.get("image") or {}
        self._allow_temperature = bool(self._chat.get("allow_temperature", False))

        # chat クライアント
        if self._provider == "azure":
            from openai import AzureOpenAI
            self._client_chat = AzureOpenAI(
                api_key=self._chat.get("api_key"),
                api_version=self._chat.get("api_version", "2024-12-01-preview"),
                azure_endpoint=self._chat.get("azure_endpoint"),
            )
            # 画像は今は未使用（将来用）
            if self._image.get("api_key") and self._image.get("azure_endpoint"):
                self._client_image = AzureOpenAI(
                    api_key=self._image.get("api_key"),
                    api_version=self._image.get("api_version", "2025-04-01-preview"),
                    azure_endpoint=self._image.get("azure_endpoint"),
                )
        else:
            from openai import OpenAI
            self._client_chat = OpenAI(api_key=self._chat.get("api_key"))

        self._deployment = self._chat.get("deployment", "o1")
        self._temperature = float(self._chat.get("temperature", 0.8))
        self._timeout = float(self._chat.get("timeout_sec", 4.0))

    def generate(self, intent, profile, recent_events):
        sys_prompt = (
            "あなたは小型お絵描きロボJedyの会話生成AIです。"
            "口調はやわらかく、1文は短め、絵を描くのが好き。"
            "出力はJSONで、keys: text(<=10字), emotion(one of: joy, calm, interest, surprise, sorry, neutral)。"
        )
        ctx = {
            "profile": profile,           # {profile:{...}, hint:str} を想定
            "intent": intent,
            "recent_events": recent_events[-5:],
        }

        # --- まずは “絶対通る” ミニマムリクエストで投げる（temperature等なし） ---
        req = {
            "messages": [
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": json.dumps(ctx, ensure_ascii=False)}
            ],
            "model": self._deployment,   # Azure: デプロイ名
        }
        
        try:
            resp = self._client_chat.chat.completions.create(**req)
        except Exception as e:
            # もし別モデル向けに温度を使いたい設定なら、リトライ（o1では到達しない）
            if self._allow_temperature:
                try:
                    req_retry = dict(req)  # shallow copy
                    req_retry["temperature"] = self._temperature
                    resp = self._client_chat.chat.completions.create(**req_retry)
                except Exception as e2:
                    raise e2
            else:
                raise e

        txt = resp.choices[0].message.content.strip()
        
        # JSONでない返答に備えて安全化
        try:
            obj = json.loads(txt)
        except Exception:
            obj = {"text": txt, "emotion": "neutral"}
            
        text = str(obj.get("text","")).strip()
        emotion = str(obj.get("emotion","neutral")).strip()

        if not text:
            raise ValueError("empty text")

        # 正規化
        mapping = {"happy":"joy", "relaxed":"calm"}
        emotion = mapping.get(emotion, emotion)
        if emotion not in ["joy","calm","interest","surprise","sorry","neutral"]:
            emotion = "neutral"
        return text, emotion


class ReactionRouter:
    def __init__(self):
        rospy.init_node("reaction_router")
        self.lock = Lock()

        rospy.Subscriber("/ui/speak_lock", String, _lock_cb, queue_size=1)

        pkg_root = RosPack().get_path("techrie_demo")
        default_map = os.path.join(pkg_root, "config", "reactions.yaml")
        self.map_path = rospy.get_param("~map_file", default_map)
        self.cooldown = float(rospy.get_param("~cooldown_sec", 1.0))
        self.use_gpt = bool(rospy.get_param("~use_gpt", False))
        self.autoreload_sec = float(rospy.get_param("~autoreload_sec", 0.0))

        self._load_map()

        self.pub_txt = rospy.Publisher("/robot_text", String, queue_size=10)
        self.pub_emo = rospy.Publisher("/emotion/set", String, queue_size=10)

        self.last_time  = {k: 0.0 for k in SUPPORTED}
        self.last_text  = {k: ""  for k in SUPPORTED}  # 直前の出力（簡易リピート抑制）
        self.profile    = _read_profile(pkg_root)
        self.db_path = os.path.expanduser("~/.ros/techrie_demo/diary.db")

        # GPT 初期化（use_gpt=true のときのみ）
        self.gpt = None
        if self.use_gpt:
            cfg_file = os.path.join(pkg_root, "config", "gpt_api.yaml")
            try:
                self.gpt = GPTPhraseGen(cfg_path=cfg_file)
                rospy.loginfo("reaction_router: GPT ready (provider=azure, deployment=%s)", self.gpt._deployment)
            except Exception as e:
                rospy.logwarn("reaction_router: GPT init failed: %s", e)
                self.use_gpt = False

        for key in SUPPORTED:
            rospy.Subscriber(f"/human/{key}", String, self._mk_cb(key), queue_size=30)
        rospy.loginfo("reaction_router: map=%s cooldown=%.2fs use_gpt=%s", self.map_path, self.cooldown, self.use_gpt)

        if self.autoreload_sec > 0:
            self._schedule_reload()

    def _schedule_reload(self):
        if rospy.is_shutdown(): return
        try:
            self._load_map()
        except Exception as e:
            rospy.logwarn("reaction_router: reload failed: %s", e)
        Timer(self.autoreload_sec, self._schedule_reload).start()

    def _load_map(self):
        try:
            with open(self.map_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            rospy.logwarn("reaction_router: cannot read map %s (%s). using empty.", self.map_path, e)
            data = {}
        if isinstance(data.get("cooldown_sec"), (int, float)):
            self.cooldown = float(data["cooldown_sec"])
        self.table = {k: v for k, v in data.items() if k in SUPPORTED}
        for k in SUPPORTED:
            self.table.setdefault(k, {})

    def _fallback_yaml(self, key):
        row = self.table.get(key, {})
        text = _pick_yaml_line(row.get("text"))
        if not text:
            defaults = {
                "praise":"ありがとう！",
                "pet":"なでなで…",
                "offer_food":"いただきます！",
                "show_art":"わぁ！",
                "invite":"いっしょに描こう！OK?",
                "greeting_hello":"こんにちは！",
                "greeting_goodnight":"おやすみなさい",
                "joke":"えへへ",
                "sorry":"ごめんね",
            }
            text = defaults.get(key, "うん！")
        emotion = row.get("emotion", "neutral")
        return text, emotion

    def _emit(self, text, emotion):
        if _speak_lock:
            rospy.loginfo("reaction_router: speak locked by %s", _speak_lock)
            return
        if text:
            self.pub_txt.publish(String(text))
        if emotion:
            self.pub_emo.publish(String(emotion))

    def _mk_cb(self, key):
        def cb(_msg: String):
            with self.lock:
                now = time.time()
                if now - self.last_time.get(key, 0.0) < self.cooldown:
                    return
                self.last_time[key] = now

                row = self.table.get(key, {})
                gpt_hint = row.get("gpt_hint", "")

                text, emotion = None, None
                used = "yaml"

                if self.use_gpt and self.gpt:
                    try:
                        # recent は存在すれば使う、なければ空配列
                        recent = _read_recent_events(self.db_path, limit=8) if os.path.exists(self.db_path) else []
                        # GPTへ渡すコンテキストは「プロフィール＋インテント＋ヒント」の最小構成
                        ctx_profile = {
                            "name":  self.profile.get("name","Jedy"),
                            "likes": self.profile.get("likes",["orange","drawing"]),
                            "style": self.profile.get("style","friendly")
                        }
                        text, emotion = self.gpt.generate(
                            intent=key,
                            profile={"profile": ctx_profile, "hint": gpt_hint},
                            recent_events=recent
                        )
                        # 簡易リピート抑制（前回と同じならフォールバック）
                        if text == self.last_text.get(key, ""):
                            raise RuntimeError("same text as previous")
                        used = "gpt"
                    except Exception as ex:
                        rospy.logwarn("reaction_router: gpt failed (%s), fallback yaml", ex)
                        text, emotion = self._fallback_yaml(key)
                        used = "yaml"
                else:
                    text, emotion = self._fallback_yaml(key)
                    used = "yaml"

                self.last_text[key] = text
                self._emit(text, emotion)
                rospy.loginfo("reaction_router: %s -> (%s) text='%s' emo='%s'", key, used, text, emotion)
        return cb

def main():
    ReactionRouter()
    rospy.spin()

if __name__ == "__main__":
    main()
