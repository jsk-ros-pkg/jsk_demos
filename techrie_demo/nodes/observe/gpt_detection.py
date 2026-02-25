#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json, time, base64, io, os, random
import requests
import rospkg
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage

class GPTDetector:
    def __init__(self):
        rospy.init_node("gpt_detection")

        # ===== Params =====
        # 画像を送るか（o1等テキスト専用なら false に）
        self.vision_enabled   = bool(rospy.get_param("~vision_enabled", True))
        self.min_interval_sec = float(rospy.get_param("~min_interval_sec", 15.0))
        self.timeout_sec      = float(rospy.get_param("~timeout_sec", 10.0))
        self.max_retries      = int(rospy.get_param("~max_retries", 2))
        self.backoff_sec      = float(rospy.get_param("~backoff_sec", 1.5))
        self.max_width        = int(rospy.get_param("~max_image_width", 640))
        self.jpeg_quality     = int(rospy.get_param("~jpeg_quality", 70))
        
        # 置き換え後（日本語版）:
        self.system_prompt = rospy.get_param(
            "~system_prompt",
            "あなたは共同お絵描きロボットの相手にふさわしい、短くて優しい反応を提案するアシスタントです。"
            "ロボットのプロフィール／内的状態／人の状態／（必要なら）画像を踏まえ、場を乱さず自然なふるまいを出力してください。")
        
        self.user_prompt_tpl = rospy.get_param(
            "~user_prompt_tpl",
            """次の情報（profile, robot_state, human_state）を参考に、その場に合う一言を1つ提案してください。
            必ず **JSONのみ** を返してください。スキーマは以下です：
            {
            "utterance": "日本語の短い一言（10文字程度・子供のように・敬語はなし）",
            "intent": "greet|praise|ask|invite|comment",
            "intent_ja": "挨拶/称賛/質問/誘い/コメント",
            "priority": "0.2〜0.8 の数値（高いほど今すぐ言いたい）",
            "target": {"type":"label","label":"left_top|center_top|right_top|left_center|center_center|right_center|left_bottom|center_bottom|right_bottom"}
            }
            - 必ず上記キーを含め、JSON以外の文章は一切出力しないこと。
            - 迷った場合は `priority` を低め（0.2〜0.4）にし、`utterance` は控えめに。
            """
        )
        self.publish_topic     = rospy.get_param("~publish_topic", "/interest/suggestion")

        # gating（衝突回避）
        self.speak_lock_topic  = rospy.get_param("~speak_lock_topic", "/ui/speak_lock")
        self.painting_topic    = rospy.get_param("~painting_topic",   "/painting_active")
        self.camera_topic      = rospy.get_param("~camera_topic",     "/camera/color/image_raw")
        self.robot_state_topic = rospy.get_param("~robot_state_topic","/robot/state")  # JSON(String) 任意
        self.human_state_topic = rospy.get_param("~human_state_topic","/human_state")  # JSON(String) 任意

        # ===== API config (techrie_demo/config/gpt_api.yaml) =====
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("techrie_demo")
        api_yaml = os.path.join(pkg_path, "config", "gpt_api.yaml")
        with open(api_yaml, "r") as f:
            cfg = yaml_safe_load(f.read())
        chat_cfg = cfg.get("chat", {})
        provider = (cfg.get("provider") or "azure").lower()

        self.api_key      = chat_cfg.get("api_key", "")
        self.api_version  = chat_cfg.get("api_version", "2024-12-01-preview")
        self.deployment   = chat_cfg.get("deployment", "o1")
        self.endpoint     = chat_cfg.get("azure_endpoint", "").rstrip("/")
        # Azure Chat Completions endpoint
        self.chat_url     = f"{self.endpoint}/openai/deployments/{self.deployment}/chat/completions?api-version={self.api_version}"

        # プロフィール（config/Jedy.json）
        prof_path = os.path.join(pkg_path, "config", "Jedy.json")
        try:
            with open(prof_path, "r") as f:
                self.profile = json.load(f)
        except Exception:
            self.profile = {}

        # ===== IO =====
        self.pub = rospy.Publisher(self.publish_topic, String, queue_size=10)
        rospy.Subscriber(self.speak_lock_topic, String, self._cb_lock, queue_size=1)
        rospy.Subscriber(self.painting_topic,   Bool,   self._cb_paint, queue_size=1)
        rospy.Subscriber(self.camera_topic,     Image,  self._cb_img,  queue_size=1)
        rospy.Subscriber(self.robot_state_topic,String, self._cb_robot, queue_size=1)
        rospy.Subscriber(self.human_state_topic,String, self._cb_human, queue_size=1)

        self.bridge = CvBridge()
        self.last_img = None
        self.robot_state = {}
        self.human_state = {}
        self.locked = False
        self.painting = False
        self.last_time = 0.0

        # モデルが画像非対応っぽい場合は自動で vision 無効化（デプロイ名が 'o1' を含む等）
        if "o1" in self.deployment.lower():
            self.vision_enabled = bool(rospy.get_param("~vision_enabled", False))

        rospy.Timer(rospy.Duration(self.min_interval_sec), self._tick, oneshot=False)
        rospy.loginfo("gpt_detection: ready (vision=%s, interval=%.1fs, timeout=%.1fs)",
                      self.vision_enabled, self.min_interval_sec, self.timeout_sec)

    # ---------- callbacks ----------
    def _cb_lock(self, s: String):  self.locked = bool((s.data or "").strip())
    def _cb_paint(self, b: Bool):   self.painting = bool(b.data)

    def _cb_img(self, msg: Image):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 軽量化（幅を max_width に収める）
            h, w = cv.shape[:2]
            if w > self.max_width:
                scale = float(self.max_width) / float(w)
                nh, nw = int(h*scale), int(w*scale)
                import cv2
                cv = cv2.resize(cv, (nw, nh), interpolation=cv2.INTER_AREA)
            # JPEGに圧縮 → base64
            pil = PILImage.fromarray(cv[:, :, ::-1])  # BGR→RGB
            buf = io.BytesIO()
            pil.save(buf, format="JPEG", quality=self.jpeg_quality, optimize=True)
            self.last_img = base64.b64encode(buf.getvalue()).decode("utf-8")
        except Exception:
            self.last_img = None

    def _cb_robot(self, s: String):
        try: self.robot_state = json.loads(s.data or "{}")
        except Exception: self.robot_state = {}

    def _cb_human(self, s: String):
        try: self.human_state = json.loads(s.data or "{}")
        except Exception: self.human_state = {}

    # ---------- core ----------
    def _tick(self, _evt):
        # ゲート：発話ロック or 描画中は完全スキップ（API叩かない）
        if self.locked or self.painting:
            return

        now = time.time()
        if (now - self.last_time) < self.min_interval_sec:
            return
        self.last_time = now

        sys_txt = self.system_prompt
        usr = {
            "profile": self.profile,
            "robot_state": self.robot_state,
            "human_state": self.human_state,
        }
        usr_txt = self.user_prompt_tpl + "\n\n" + json.dumps(usr, ensure_ascii=False)

        # 呼び出し
        try:
            if self.vision_enabled and self.last_img:
                out = self._call_chat_with_image(sys_txt, usr_txt, self.last_img)
            else:
                out = self._call_chat_text_only(sys_txt, usr_txt)
        except Exception as e:
            rospy.logwarn("gpt_detection: call error: %s", e)
            return

        if not out:
            return

        # publish
        self.pub.publish(String(json.dumps(out, ensure_ascii=False)))

    def _headers(self):
        return {
            "api-key": self.api_key,
            "Content-Type": "application/json"
        }

    def _call_chat_text_only(self, system_text, user_text):
        body = {
            "messages": [
                {"role": "system", "content": system_text},
                {"role": "user",   "content": user_text}
            ]
        }
        return self._post_with_retry(body)

    def _call_chat_with_image(self, system_text, user_text, b64_image):
        body = {
            "messages": [
                {"role": "system", "content": system_text},
                {"role": "user", "content": [
                    {"type": "text", "text": user_text},
                    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + b64_image}}
                ]}
            ]
        }
        return self._post_with_retry(body)

    def _post_with_retry(self, body):
        # リトライ＆指数バックオフ、タイムアウト拡張
        for attempt in range(self.max_retries + 1):
            try:
                r = requests.post(self.chat_url, headers=self._headers(),
                                  json=body, timeout=self.timeout_sec)
                if r.status_code == 200:
                    data = r.json()
                    # Azure Chat Completions 形式
                    msg = data["choices"][0]["message"]["content"]
                    return self._safe_parse(msg)
                else:
                    # 4xx/5xx はメッセージを出して次回
                    rospy.logwarn("gpt_detection: status %s %s", r.status_code, r.text[:200])
            except requests.exceptions.ReadTimeout:
                rospy.logwarn("gpt_detection: read timeout (%.1fs)", self.timeout_sec)
            except Exception as e:
                rospy.logwarn("gpt_detection: request error: %s", e)
            # backoff
            time.sleep(self.backoff_sec * (2 ** attempt))
        return None

    def _safe_parse(self, content):
        """
        期待JSON:
        {
          "utterance": "わぁ！きれい",
          "intent": "comment",
          "priority": 0.5,
          "target": {"type":"label","label":"center_center"}
        }
        """
        content = (content or "").strip()
        # LLMがテキストを混ぜた場合に備え、最初の { … } を抜き出す
        start = content.find("{")
        end   = content.rfind("}")
        if start >= 0 and end > start:
            content = content[start:end+1]
        try:
            d = json.loads(content)
        except Exception:
            # 最低限のフォールバック
            return {"utterance": content[:60], "intent": "comment", "priority": 0.3,
                    "target": {"type":"label","label":"center_center"}}
        # 型の整形
        if "priority" in d:
            try: d["priority"] = float(d["priority"])
            except: d["priority"] = 0.3
        if "target" not in d:
            d["target"] = {"type":"label","label":"center_center"}
        return d

def yaml_safe_load(text):
    # pyyaml の Loader 警告回避（安全なローダを使用）
    import yaml
    return yaml.load(text, Loader=yaml.SafeLoader)

def main():
    GPTDetector()
    rospy.spin()

if __name__ == "__main__":
    main()
