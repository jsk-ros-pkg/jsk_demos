#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, yaml, os, json, time, math, random
from rospkg import RosPack
from std_msgs.msg import String, Bool
from techrie_demo.msg import DesireState, InteractionEvent

ACTIONS = ["invite","paint","show","eat","observe","idle"]

def softmax(xs):
    m = max(xs) if xs else 0.0
    ex = [math.exp(x-m) for x in xs]
    s = sum(ex) or 1.0
    return [e/s for e in ex]

class GPTClient(object):
    def __init__(self, cfg_path):
        self.client = None
        self.deployment = "o1"
        self.allow_temperature = False
        if not os.path.exists(cfg_path):
            raise RuntimeError("gpt_api.yaml not found: %s" % cfg_path)
        with open(cfg_path,"r",encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        chat = cfg.get("chat") or {}
        provider = (cfg.get("provider") or "azure").lower()
        self.deployment = chat.get("deployment","o1")
        self.allow_temperature = bool(chat.get("allow_temperature", False))
        if provider == "azure":
            from openai import AzureOpenAI
            self.client = AzureOpenAI(
                api_key=chat.get("api_key"),
                api_version=chat.get("api_version","2024-12-01-preview"),
                azure_endpoint=chat.get("azure_endpoint"),
            )
        else:
            from openai import OpenAI
            self.client = OpenAI(api_key=chat.get("api_key"))

    def decide(self, context):
        sys_prompt = (
            "あなたは小型お絵描きロボJedyの行動選択AIです。"
            "候補: invite, paint, show, eat, observe, idle。"
            "出力はJSONで {\"action\":\"...\"} だけ。"
            "allow_paint が false のときは paint は選ばない。"
            "人の入力が強い場合は優先。ただし同じ行動を連続しすぎない。"
        )
        req = {"messages":[
                {"role":"system","content":sys_prompt},
                {"role":"user","content": json.dumps(context, ensure_ascii=False)}
              ],
              "model": self.deployment}
        resp = self.client.chat.completions.create(**req)
        txt = resp.choices[0].message.content.strip()
        try:
            obj = json.loads(txt); a = obj.get("action","")
            return a if a in ACTIONS else None
        except Exception:
            return None

class PolicySelector(object):
    def __init__(self):
        rospy.init_node("policy_selector")

        # ===== パラメータ（ゆっくり＆安定寄りデフォルト） =====
        self.rate_hz           = float(rospy.get_param("~rate_hz", 0.5))     # 2秒に1回
        self.ambiguity_delta   = float(rospy.get_param("~ambiguity_delta", 0.25))
        self.min_dwell_sec     = float(rospy.get_param("~min_dwell_sec", 6.0))
        self.cooldown_invite   = float(rospy.get_param("~cooldown_invite_sec", 10.0))
        self.cooldown_paint    = float(rospy.get_param("~cooldown_paint_sec", 6.0))
        self.hboost_decay      = float(rospy.get_param("~human_boost_decay", 0.96))
        self.hboost_add        = float(rospy.get_param("~human_boost_add", 0.6))
        self.alpha_W           = float(rospy.get_param("~alpha_W", 0.7))
        self.beta_D            = float(rospy.get_param("~beta_D", 0.8))
        self.gamma_H           = float(rospy.get_param("~gamma_H", 0.9))
        self.epsilon           = float(rospy.get_param("~epsilon", 0.02))
        self.use_gpt           = bool(rospy.get_param("~use_gpt", True))
        self.gpt_every_sec     = float(rospy.get_param("~gpt_every_sec", 15.0))
        self.allow_paint_gate  = True

        # Invite結果でのゲート
        self.block_paint_after_invite_ng_sec  = float(rospy.get_param("~block_paint_after_invite_ng_sec", 8.0))
        self.prefer_paint_after_invite_ok_sec = float(rospy.get_param("~prefer_paint_after_invite_ok_sec", 10.0))
        self.block_paint_until  = 0.0
        self.prefer_paint_until = 0.0

        # 発話ロックを尊重（invite/paint 実行中に無闇に切り替えない）
        self.respect_lock = bool(rospy.get_param("~respect_speak_lock", True))
        self.locked_by = ""

        # ===== 入出力 =====
        self.pub_choice  = rospy.Publisher("/policy/next_action", String, queue_size=10)
        self.sub_started = rospy.Subscriber("/system/started", Bool, lambda m: setattr(self,"started",bool(m.data)), queue_size=1)
        self.started     = False

        # 欲求D（EMAで平滑）:contentReference[oaicite:9]{index=9} :contentReference[oaicite:10]{index=10}
        self.D = {a:0.0 for a in ACTIONS}
        rospy.Subscriber("/desire/state", DesireState, self._cb_desire, queue_size=10)

        # 嗜好W（profile_manager により随時更新）:contentReference[oaicite:11]{index=11}
        self.W = {a: 0.0 for a in ACTIONS}
        rospy.Subscriber("/policy/weights", String, self._cb_weights, queue_size=10)

        # 人入力ブースト
        self.H = {a: 0.0 for a in ACTIONS}
        self._setup_human_hooks()

        # allow_paint ゲート
        rospy.Subscriber("/allow_paint", Bool, lambda m: setattr(self,"allow_paint_gate", bool(m.data)), queue_size=1)

        # ロック購読
        rospy.Subscriber("/ui/speak_lock", String, self._cb_lock, queue_size=1)

        # イベント購読（Invite結果でゲート）:contentReference[oaicite:12]{index=12}
        rospy.Subscriber("/interaction_events", InteractionEvent, self._cb_event, queue_size=50)

        # クールダウン・滞在
        self.last_time_action = {a: 0.0 for a in ACTIONS}
        self.last_chosen = "idle"
        self.last_switch_ts = 0.0

        # GPT
        self.gpt = None
        self.last_gpt_ts = 0.0
        if self.use_gpt:
            try:
                cfg = os.path.join(RosPack().get_path("techrie_demo"), "config", "gpt_api.yaml")
                self.gpt = GPTClient(cfg)
                rospy.loginfo("policy_selector: GPT ready (deployment=%s)", self.gpt.deployment)
            except Exception as e:
                rospy.logwarn("policy_selector: GPT init failed: %s", e)
                self.use_gpt = False

        rospy.loginfo("policy_selector started: rate=%.2fHz, min_dwell=%.1fs, gpt_every=%.1fs",
                      self.rate_hz, self.min_dwell_sec, self.gpt_every_sec)

    # ===== callbacks =====
    def _cb_desire(self, msg: DesireState):
        try:
            vals = list(msg.values)
            for i,a in enumerate(ACTIONS):
                self.D[a] = 0.9*self.D[a] + 0.1*float(vals[i])
        except Exception:
            pass

    def _cb_weights(self, msg: String):
        try:
            obj = json.loads(msg.data)
            for a in ACTIONS:
                if a in obj: self.W[a] = float(obj[a])
        except Exception:
            pass

    def _setup_human_hooks(self):
        def mk(action, gain):
            def cb(_): self.H[action] += gain
            return cb
        rospy.Subscriber("/human/praise",     String, mk("show",    self.hboost_add), queue_size=10)
        rospy.Subscriber("/human/pet",        String, mk("idle",    self.hboost_add*0.7), queue_size=10)
        rospy.Subscriber("/human/offer_food", String, mk("eat",     self.hboost_add), queue_size=10)
        rospy.Subscriber("/human/show_art",   String, mk("observe", self.hboost_add), queue_size=10)
        rospy.Subscriber("/human/invite",     String, mk("invite",  self.hboost_add), queue_size=10)

    def _cb_lock(self, msg: String):
        self.locked_by = (msg.data or "").strip()

    def _cb_event(self, e: InteractionEvent):
        now = time.time()
        if e.event_type == "INVITE_NG":
            self.block_paint_until = now + self.block_paint_after_invite_ng_sec
        elif e.event_type == "INVITE_OK":
            self.prefer_paint_until = now + self.prefer_paint_after_invite_ok_sec

    # ===== policy =====
    def _utility(self):
        # 短期ブーストの減衰
        for a in ACTIONS: self.H[a] *= self.hboost_decay

        U = {}
        for a in ACTIONS:
            U[a] = self.alpha_W*self.W[a] + self.beta_D*self.D[a] + self.gamma_H*self.H[a]

        now = time.time()
        # ゲート
        if not self.allow_paint_gate: U["paint"] = -1e3
        if now - self.last_time_action["invite"] < self.cooldown_invite: U["invite"] -= 1.0
        if now - self.last_time_action["paint"]  < self.cooldown_paint:  U["paint"]  -= 0.5

        # Invite NG 後は PAINT 抑制、Invite OK 後は PAINT を少し後押し
        if now < self.block_paint_until:  U["paint"] -= 2.0
        if now < self.prefer_paint_until: U["paint"] += 0.6

        # 最小滞在：直前行動に粘り
        if now - self.last_switch_ts < self.min_dwell_sec:
            U[self.last_chosen] += 0.6
        return U

    def _choose(self, U):
        # ロック中は切替しない（実行ノードが主導）
        if self.respect_lock and self.locked_by:
            return self.last_chosen

        # ε-greedy
        if random.random() < self.epsilon: return random.choice(ACTIONS)

        items = sorted(U.items(), key=lambda kv: kv[1], reverse=True)
        best, second = items[0], items[1]
        ambiguous = (best[1] - second[1]) < self.ambiguity_delta

        # GPT は曖昧時 or インターバル経過時のみ、かつ最小滞在を満たしたとき
        use_gpt_now = False
        t = time.time()
        if self.use_gpt and (ambiguous or (t - self.last_gpt_ts >= self.gpt_every_sec)):
            if t - self.last_switch_ts >= self.min_dwell_sec and not (self.respect_lock and self.locked_by):
                use_gpt_now = True

        choice = best[0]
        if use_gpt_now and self.gpt:
            ctx = {
                "last_action": self.last_chosen,
                "allow_paint": bool(self.allow_paint_gate),
                "top_k": items[:3],
                "candidate_scores": U,
                "human_recent": [a for a,v in self.H.items() if v > 0.3],
            }
            try:
                a = self.gpt.decide(ctx)
                if a in ACTIONS:
                    choice = a
                    self.last_gpt_ts = time.time()
                    rospy.loginfo("policy_selector: GPT override -> %s", a)
            except Exception as e:
                rospy.logwarn("policy_selector: GPT decide failed: %s", e)
        return choice

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not getattr(self, "started", False):
                r.sleep(); continue
            U = self._utility()
            choice = self._choose(U)
            self.pub_choice.publish(String(choice))
            now = time.time()
            if choice != self.last_chosen:
                self.last_chosen = choice
                self.last_switch_ts = now
            self.last_time_action[choice] = now
            r.sleep()

def main():
    node = PolicySelector(); node.spin()

if __name__ == "__main__":
    main()
