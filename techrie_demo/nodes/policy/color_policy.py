#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json, os, time, random
from std_msgs.msg import String
from techrie_demo.msg import InteractionEvent

def expand(p): return os.path.expanduser(p)

class ColorPolicy:
    def __init__(self):
        rospy.init_node('color_policy')

        # palette（候補色）
        self.palette = rospy.get_param('~palette', ['red','blue','yellow','green','orange','purple','pink'])
        self.epsilon = rospy.get_param('~epsilon', 0.25)  # 探索率
        self.human_override_timeout = rospy.get_param('~human_override_timeout', 30.0)  # 手動操作からの猶予
        self.color_cooldown_sec = rospy.get_param('~color_cooldown_sec', 10.0)          # 連投抑止
        self.tick_hz = rospy.get_param('~rate', 0.5)  # 2秒ごとに提案

        # favorite 色（Jedy.json から）
        jedy_path = expand(rospy.get_param('~jedy_profile', ''))
        self.favorite = self._load_favorite(jedy_path)

        # 状態
        self.recent_success = {c: 0.0 for c in self.palette}  # 指数減衰の成功カウント
        self.cooldown_until = {c: 0.0 for c in self.palette}
        self.last_manual_color = None
        self.last_manual_time = 0.0
        self.last_published = None

        # IO
        rospy.Subscriber('/interaction_events', InteractionEvent, self.on_event, queue_size=100)
        rospy.Subscriber('/paint_color', String, self.on_color_in, queue_size=10)
        rospy.Subscriber('/profile/weights', String, self.on_weights, queue_size=1)  # 使うなら拡張
        self.pub = rospy.Publisher('/paint_color_policy', String, queue_size=1, latch=True)

    def _load_favorite(self, path):
        try:
            if path and os.path.exists(path):
                with open(path, 'r') as f:
                    data = json.load(f)
                fav = (data.get('favorite_color') or '').lower()
                # 例: 'orange', 'orange-red' などの場合に先頭を使う
                if isinstance(fav, str) and fav:
                    base = fav.split('/')[0].split('-')[0].strip()
                    return base
        except Exception as e:
            rospy.logwarn("color_policy: failed to read Jedy.json: %s", e)
        return ''  # 既定

    def on_event(self, e: InteractionEvent):
        # ペイント成功色を指数減衰で記録
        if e.event_type == 'PAINT_END':
            try:
                meta = json.loads(e.meta_json) if e.meta_json else {}
            except Exception:
                meta = {}
            col = (meta.get('color') or '').lower()
            if not col:
                return
            for k in list(self.recent_success.keys()):
                # 時間経過で自然減衰（ここはイベントが来た時だけでもOK）
                self.recent_success[k] *= 0.95
            if col not in self.recent_success:
                # パレット外の色が来た場合、追加して追跡
                self.recent_success[col] = 0.0
                if col not in self.palette:
                    self.palette.append(col)
                    self.cooldown_until[col] = 0.0
            self.recent_success[col] += 1.0
            # 連投抑止：成功直後は少し冷却
            self.cooldown_until[col] = time.time() + max(2.0, self.color_cooldown_sec * 0.5)

    def on_color_in(self, s: String):
        # 誰が出したかはトピックでは判別できないので、
        # 値が変わったら「手動変更があった」とみなして一定時間は上書きしない
        col = (s.data or '').lower()
        if col and col != self.last_published:
            self.last_manual_color = col
            self.last_manual_time = time.time()

    def on_weights(self, s: String):
        # 今は未使用。将来：with_peopleが高い日は暖色をやや優先、などの拡張に利用可能。
        pass

    def score(self, c):
        """favorite ボーナスを弱く、クールダウン中は強めに抑制。"""
        now = time.time()
        # 完全一致のみボーナス（部分一致はやめる）
        fav_bonus = (getattr(self, 'favorite_bonus', 0.05)
                     if (getattr(self, 'favorite', '') and getattr(self, 'favorite') == c)
                     else 0.0)
        # 最近の成功（存在しなければ 0）
        succ = getattr(self, 'recent_success', {}).get(c, 0.0)
        # クールダウン中はペナルティ
        penalty = 0.3 if getattr(self, 'cooldown_until', {}).get(c, 0.0) > now else 0.0
        return fav_bonus + 0.12 * succ - penalty


    def choose(self):
        """ε で完全ランダム、そうでなければソフトマックス抽選で偏りを緩和。"""
        import math
        now = time.time()

        # 既存プロパティが無い場合でも動くようにデフォルト化
        if not hasattr(self, 'cooldown_until'): self.cooldown_until = {}
        if not hasattr(self, 'recent_success'): self.recent_success = {}
        if not hasattr(self, 'palette'): self.palette = ['red','blue','yellow','green','orange','purple','pink']
        
        # 手動操作が最近あれば見守り（手動優先）
        last_manual = getattr(self, 'last_manual_time', 0.0)
        human_to = getattr(self, 'human_override_timeout', 0.0)
        if now - last_manual < human_to:
            return None

        epsilon = getattr(self, 'epsilon', 0.25)
        temperature = getattr(self, 'temperature', 0.9)
        
        # ε 探索：クールダウン外から完全ランダム
        if random.random() < epsilon:
            candidates = [c for c in self.palette if self.cooldown_until.get(c,0.0) <= now] or list(self.palette)
            choice = random.choice(candidates)
        else:
            # ソフトマックス抽選（クールダウン中は除外）
            actives = [(c, self.score(c)) for c in self.palette if self.cooldown_until.get(c,0.0) <= now]
            if not actives:
                actives = [(c, self.score(c)) for c in self.palette]
            m = max(s for _, s in actives) if actives else 0.0
            weights = [math.exp((s - m) / max(1e-6, temperature)) for _, s in actives]
            total = sum(weights)
            if total <= 0:
                choice = random.choice([c for c, _ in actives])
            else:
                r = random.random() * total
                acc = 0.0
                choice = actives[-1][0]
                for (c, _), w in zip(actives, weights):
                    acc += w
                    if r <= acc:
                        choice = c
                        break

        # クールダウン更新（デフォルト 12s に少し伸ばす）
        cd = getattr(self, 'color_cooldown_sec', 12.0)
        self.cooldown_until[choice] = now + cd
        return choice


    def spin(self):
        rate = rospy.Rate(self.tick_hz)
        while not rospy.is_shutdown():
            choice = self.choose()
            if choice:
                self.last_published = choice
                self.pub.publish(String(choice))
                rospy.loginfo("color_policy: -> %s (fav=%s, succ=%.2f)",
                              choice, self.favorite, self.recent_success.get(choice,0.0))
            rate.sleep()

def main():
    cp = ColorPolicy()
    cp.spin()

if __name__ == '__main__':
    main()
