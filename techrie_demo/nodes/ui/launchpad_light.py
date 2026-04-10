#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launchpad X を Programmer Mode で直接光らせるノード
- 入力:
    /emotion/set (std_msgs/String)    : joy/calm/interest/surprise/sorry/neutral
    /human/* (std_msgs/String)        : 人入力直後に短いパルス（白フラッシュ）
- 動作:
    8x8全パッドと周辺ボタンを感情色で点灯。パルス中は白→元色。
- 依存:
    pip install mido python-rtmidi
"""
import rospy
import time
from std_msgs.msg import String
import mido

# ====== パレット番号（Novationの内蔵カラーインデックス） ======
# 実機の見え方によって微調整してください。
PALETTE = {
    "white": 3,
    "red": 5,
    "orange": 9,
    "yellow": 13,
    "green": 21,
    "cyan": 37,
    "blue": 45,
    "purple": 53,
    "pink": 57,
    "amber": 11,
}

# 感情→パレット色の対応（expression_hub と揃えるのは後でOK）
EMOTION_TO_COLOR = {
    "joy":      "orange",
    "calm":     "blue",
    "interest": "purple",
    "surprise": "yellow",
    "sorry":    "cyan",
    "neutral":  "white",
}

# 機能ボタンの CC マップ（Programmer Mode）
TOP_CC    = [91,92,93,94,95,96,97,98,99]  # 上列
RIGHT_CC  = [19,29,39,49,59,69,79,89]     # 右側列
BOTTOM_CC = [8,12,13,14,15,16,17,18]      # 下段

def find_launchpad_port():
    """Launchpad の出力ポート候補を探す（LPX MIDI優先 / DAW除外）"""
    names = mido.get_output_names()
    preferred = [n for n in names if 'Launchpad' in n and 'DAW' not in n and ('LPX MIDI' in n or 'MIDI' in n)]
    if preferred:
        return preferred[0]
    for n in names:
        if 'Launchpad' in n and 'DAW' not in n:
            return n
    raise RuntimeError('Launchpad X の出力ポートが見つかりません。候補: {}'.format(names))

class LaunchpadX:
    def __init__(self, port_name=None, force_programmer=True, clear_on_start=True):
        if port_name is None:
            port_name = find_launchpad_port()
        self.port_name = port_name
        self.out = mido.open_output(self.port_name)
        if force_programmer:
            # Programmer Mode（必要な時だけ）
            self.out.send(mido.Message('sysex', data=[0x00,0x20,0x29,0x02,0x0C,0x0E,0x01]))
        if clear_on_start:
            self.clear()

    def clear(self):
        # 8x8
        for row in range(1,9):
            for col in range(1,9):
                note = row*10 + col
                self.out.send(mido.Message('note_on', channel=0, note=note, velocity=0))
        # 周辺
        for cc in TOP_CC + RIGHT_CC + BOTTOM_CC:
            self.out.send(mido.Message('control_change', channel=0, control=cc, value=0))

    def fill_color(self, palette_idx):
        """8x8全体と周辺ボタンを同じ色で塗る"""
        # 8x8
        for row in range(1,9):
            for col in range(1,9):
                note = row*10 + col
                self.out.send(mido.Message('note_on', channel=0, note=note, velocity=palette_idx))
        # 周辺
        for cc in TOP_CC + RIGHT_CC + BOTTOM_CC:
            self.out.send(mido.Message('control_change', channel=0, control=cc, value=palette_idx))

    def flash_all_white_once(self, white_idx, hold_sec=0.08):
        """短い白フラッシュ（人入力時パルス）。hold後は呼び出し側で元色へ戻す。"""
        # 8x8
        for row in range(1,9):
            for col in range(1,9):
                note = row*10 + col
                self.out.send(mido.Message('note_on', channel=0, note=note, velocity=white_idx))
        # 周辺
        for cc in TOP_CC + RIGHT_CC + BOTTOM_CC:
            self.out.send(mido.Message('control_change', channel=0, control=cc, value=white_idx))
        time.sleep(hold_sec)

    def close(self):
        try:
            self.clear()
        finally:
            self.out.close()

class LaunchpadEmotionLightNode:
    def __init__(self):
        rospy.init_node("launchpad_light")

        # パラメータ
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.pulse_duration = float(rospy.get_param("~pulse_duration", 0.6))  # 人入力パルスの合計時間
        self.white_hold = float(rospy.get_param("~white_hold", 0.08))         # 白表示の1回保持
        self.port_name = rospy.get_param("~midi_port", "")                    # 明示指定も可
        self.force_programmer = bool(int(rospy.get_param("~force_programmer", 1)))
        self.clear_on_start  = bool(int(rospy.get_param("~clear_on_start", 1)))
        self.lp = LaunchpadX(self.port_name or None,
                             force_programmer=self.force_programmer,
                             clear_on_start=self.clear_on_start)


        # 状態
        self.cur_emotion = "neutral"
        self.base_palette = PALETTE[EMOTION_TO_COLOR[self.cur_emotion]]
        self.pulse_until = 0.0

        # 購読
        rospy.Subscriber("/emotion/set", String, self.cb_emotion, queue_size=20)

        # 人入力でパルス
        human_topics = ["praise","pet","offer_food","show_art","invite","greeting_hello","greeting_goodnight"]
        for t in human_topics:
            rospy.Subscriber(f"/human/{t}", String, self.cb_human, queue_size=20)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("launchpad_light: MIDI port='%s', rate=%.1fHz", self.lp.port_name, self.rate_hz)

    def cb_emotion(self, msg):
        em = (msg.data or "").strip().lower()
        if em not in EMOTION_TO_COLOR:
            em = "neutral"
        self.cur_emotion = em
        self.base_palette = PALETTE[EMOTION_TO_COLOR[em]]
        # すぐに反映
        self.lp.fill_color(self.base_palette)

    def cb_human(self, _msg):
        # パルスを開始/延長
        now = time.time()
        self.pulse_until = max(self.pulse_until, now + self.pulse_duration)

    def loop(self):
        r = rospy.Rate(self.rate_hz)
        last_emotion = None
        # 初期反映
        self.lp.fill_color(self.base_palette)

        while not rospy.is_shutdown():
            now = time.time()
            if self.pulse_until > now:
                # 短い白フラッシュを繰り返す（見やすい簡易パルス）
                self.lp.flash_all_white_once(PALETTE["white"], hold_sec=self.white_hold)
                # 直後に元色に戻す
                self.lp.fill_color(self.base_palette)
            else:
                # 通常は何もしない（点灯維持）
                pass

            # emotion が変わったら即反映（cbでも反映しているが保険）
            if last_emotion != self.cur_emotion:
                self.lp.fill_color(self.base_palette)
                last_emotion = self.cur_emotion

            r.sleep()

    def on_shutdown(self):
        try:
            self.lp.clear()
        except Exception:
            pass
        try:
            self.lp.close()
        except Exception:
            pass

def main():
    node = LaunchpadEmotionLightNode()
    node.loop()

if __name__ == "__main__":
    main()
