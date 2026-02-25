#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, json, os, time
from std_msgs.msg import String
from techrie_demo.msg import InteractionEvent

# ---------- utils ----------
def expand(p): return os.path.expanduser(p)
def ensure_parent_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

def clamp01(x): return max(0.0, min(1.0, x))

# ---------- globals ----------
WEIGHTS = {'invite':0.5,'paint':0.5,'show':0.3,'eat':0.2,'idle':0.3}
SAVE = None  # paramでセット
JEDY_PATH = None

# ---------- init & update ----------
def load_initial(jedy_path, save_path):
    """既存stateがあれば読む。無ければJedy.jsonから初期重みを作る。"""
    global WEIGHTS
    # 1) 既存state.jsonがあればロード
    if save_path and os.path.exists(save_path):
        try:
            with open(save_path, 'r') as f:
                WEIGHTS.update(json.load(f))
            rospy.loginfo("profile_manager: loaded weights from %s", save_path)
            return
        except Exception as e:
            rospy.logwarn("profile_manager: failed to load %s: %s", save_path, e)

    # 2) Jedy.jsonから初期重み
    try:
        with open(jedy_path, 'r') as f:
            jedy = json.load(f)
    except Exception as e:
        rospy.logwarn("profile_manager: failed to read Jedy.json (%s). using defaults.", e)
        jedy = {}

    big5 = (jedy.get('big5') or {})
    extr = float(big5.get('extraversion', 0.7))
    open_ = float(big5.get('openness',     0.9))
    agre  = float(big5.get('agreeableness',0.8))
    neur  = float(big5.get('neuroticism',  0.3))

    WEIGHTS['invite'] = clamp01(0.4 + 0.4*extr)
    WEIGHTS['paint']  = clamp01(0.4 + 0.4*open_)
    WEIGHTS['show']   = clamp01(0.3 + 0.3*agre)
    WEIGHTS['idle']   = clamp01(0.2 + 0.3*(1.0-neur))
    WEIGHTS['eat']    = clamp01(0.2)

def save_now():
    """現在の重みを保存（ディレクトリ自動作成、失敗しても落ちない）"""
    if not SAVE: return
    try:
        ensure_parent_dir(SAVE)
        with open(SAVE, 'w') as f:
            json.dump(WEIGHTS, f)
    except Exception as e:
        rospy.logwarn("profile_manager: save failed: %s", e)

def reward(evt: InteractionEvent):
    """イベントに応じて重みを微調整（超簡易バンディット）"""
    et = evt.event_type
    meta = {}
    try:
        if evt.meta_json:
            meta = json.loads(evt.meta_json)
    except Exception:
        pass

    if et == 'INVITE_OK':
        WEIGHTS['invite'] = clamp01(WEIGHTS['invite'] + 0.15)
    elif et == 'INVITE_NG':
        WEIGHTS['invite'] = clamp01(WEIGHTS['invite'] - 0.08)
    elif et == 'PAINT_END':
        WEIGHTS['paint'] = clamp01(WEIGHTS['paint'] + 0.10)
        col = (meta.get('color') or '').lower()
        if 'orange' in col:  # 好みの色に近ければ上乗せ（例）
            WEIGHTS['paint'] = clamp01(WEIGHTS['paint'] + 0.05)
    elif et == 'SHOW_DONE':
        WEIGHTS['show'] = clamp01(WEIGHTS['show'] + 0.05)
    elif et == 'PAINT_FAIL':
        WEIGHTS['paint'] = clamp01(WEIGHTS['paint'] - 0.05)

# ---------- main ----------
def main():
    global SAVE, JEDY_PATH
    rospy.init_node('profile_manager')

    # まずparamを読み、SAVE/JEDY_PATHを決定
    DEFAULT_SAVE = expand("~/.ros/techrie_demo/profile/state.json")
    JEDY_PATH = expand(rospy.get_param('~jedy_profile', 'Jedy.json'))
    SAVE = expand(rospy.get_param('~save_path', DEFAULT_SAVE))

    # 保存先ディレクトリを先に作っておく
    ensure_parent_dir(SAVE)

    pub = rospy.Publisher('/profile/weights', String, queue_size=1, latch=True)

    # 初期化（← param 確定後に呼ぶのがポイント）
    load_initial(JEDY_PATH, SAVE)
    pub.publish(String(json.dumps(WEIGHTS, ensure_ascii=False)))
    save_now()

    def on_event(e: InteractionEvent):
        reward(e)
        pub.publish(String(json.dumps(WEIGHTS, ensure_ascii=False)))
        save_now()

    rospy.Subscriber('/interaction_events', InteractionEvent, on_event, queue_size=100)

    rate = rospy.Rate(0.2)  # 5秒に1回
    while not rospy.is_shutdown():
        pub.publish(String(json.dumps(WEIGHTS, ensure_ascii=False)))
        save_now()
        rate.sleep()

if __name__ == '__main__':
    main()

