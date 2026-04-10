#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from std_msgs.msg import String, ColorRGBA, UInt16

# ============================================================
# 設定
# ============================================================
# フォールバックのデフォルトLEDパラメータ
DEFAULT_MODE = 1
DEFAULT_BLINK = 3
DEFAULT_DURATION = 1
DEFAULT_RAINBOW_HUE = 1
DEFAULT_BRIGHTNESS = 10  # ColorRGBA.a（0-255前提）

# フォールバックの感情→RGB（0-255スケール）
EMO_COL_255 = {
    "joy":      (255, 217,   0),   # warm yellow
    "sad":      ( 51, 102, 204),   # blue
    "anger":    (255,  51,  51),   # red
    "surprise": (230, 230, 255),   # white-ish
    "interest": ( 51, 255, 153),   # mint
    "calm":     (179, 230, 230),   # pale cyan
    "want":     (255, 170,   0),   # eager: vivid amber
    "show":     (255, 102, 204),   # attention-seeking: hot pink
}

# ============================================================
# レガシー（make_emotion/make_tone）取込み
# ============================================================
USE_LEGACY = True
_em = None  # make_emotion.emotion_node インスタンス
try:
    import make_emotion
    # emotion_node は /led_mode なども含めてPublishしてくれる想定
    _em = make_emotion.emotion_node()
except Exception as e:
    rospy.logwarn("make_emotion not available: %s", e)
    USE_LEGACY = False

# make_tone はオプション（無ければ簡易トーンを流すだけ）
try:
    import make_tone
    HAVE_TONE = True
except Exception as e:
    rospy.logwarn("make_tone not available: %s", e)
    HAVE_TONE = False

# フォールバック用の簡易トーン（周波数配列）
EMO_TONE = {
    'joy':      [880, 1319, 1760],
    'sad':      [392, 330, 294],
    'anger':    [440, 660, 880],
    'surprise': [1200, 900, 1200],
    'interest': [523, 659, 784],
    'calm':     [440, 440, 440],
    'want':     [660, 784, 988, 1175, 1319],
    'show':     [880, 1175, 1047, 1175, 880],
}
DUR_MS = 160  # 1ノート長さ（ミリ秒）

# ============================================================
# 本体
# ============================================================
class ExpressionHub:
    def __init__(self):
        rospy.init_node("expression_hub")

        # 出力Publishers
        self.pub_led_rgb   = rospy.Publisher("/led_rgb", ColorRGBA, queue_size=1)
        self.pub_led_blink = rospy.Publisher("/led_blink_time", UInt16, queue_size=1)
        self.pub_led_dur   = rospy.Publisher("/led_duration", UInt16, queue_size=1)
        self.pub_led_mode  = rospy.Publisher("/led_mode", UInt16, queue_size=1)
        self.pub_led_rbow  = rospy.Publisher("/led_rainbow_delta_hue", UInt16, queue_size=1)

        self.pub_beep = rospy.Publisher("tone_sequence", String, queue_size=1)
        self.pub_txt  = rospy.Publisher("/jedy_voice", String, queue_size=1)

        # ★ 追加：現在感情の外部公開
        self.profile_prefix = rospy.get_param("~profile_params_prefix", "/profile")
        self.pub_robot_emo  = rospy.Publisher("/robot_emotion", String, queue_size=10)

        # 入力Subscribers
        rospy.Subscriber("/emotion/set", String, self.on_emotion, queue_size=10)
        rospy.Subscriber("/robot_text",  String, self.on_text,    queue_size=10)

        rospy.loginfo("expression_hub ready (legacy=%s, profile_prefix=%s)", USE_LEGACY, self.profile_prefix)

    # ----- テキストはそのまま表示へ -----
    def on_text(self, m: String):
        txt = (m.data or "")
        self.pub_txt.publish(txt)

    # ----- 感情イベント -----
    def on_emotion(self, m: String):
        emo = (m.data or "").strip().lower()
        if not emo:
            return

        # --- LED 制御 ---
        if USE_LEGACY and _em and hasattr(_em, "color_change"):
            # レガシー側に一任（/led_mode 等も含めて発行してくれる）
            try:
                _em.color_change(emo)
            except Exception as e:
                rospy.logwarn("legacy color_change failed (%s). fallback path.", e)
                self._fallback_led(emo)
        else:
            self._fallback_led(emo)

        # --- 音（トーン） ---
        self._publish_tone(emo)

        # ★ 追加：現在の感情を /profile/emotion に保存し、可視化用に publish
        try:
            rospy.set_param(f"{self.profile_prefix}/emotion", emo)
        except Exception as e:
            rospy.logwarn("expression_hub: failed to set /profile/emotion: %s", e)
        try:
            self.pub_robot_emo.publish(emo)
        except Exception:
            pass

        rospy.loginfo("expression_hub: emotion set -> %s", emo)

    # ----- フォールバックLED -----
    def _fallback_led(self, emo: str):
        r, g, b = EMO_COL_255.get(emo, EMO_COL_255["interest"])
        color = ColorRGBA()
        color.r = int(r)
        color.g = int(g)
        color.b = int(b)
        color.a = int(DEFAULT_BRIGHTNESS)  # 0-255前提

        # ドライバがモード等を必要とするケースに合わせて、最低限の既定値を送る
        self.pub_led_blink.publish(UInt16(DEFAULT_BLINK))
        self.pub_led_dur.publish(UInt16(DEFAULT_DURATION))
        self.pub_led_mode.publish(UInt16(DEFAULT_MODE))
        self.pub_led_rbow.publish(UInt16(DEFAULT_RAINBOW_HUE))
        self.pub_led_rgb.publish(color)

    # ----- トーンPublish（make_tone優先、無ければ簡易） -----
    def _publish_tone(self, emo: str):
        tones_list = None

        if HAVE_TONE:
            try:
                # 既存API（戻りが list[dict{freq,duration}] 想定）
                tones_list = make_tone.generate_tone_sequence_by_emotion(
                    word=emo, emotion=emo
                )
                if not isinstance(tones_list, list) or not tones_list or "freq" not in tones_list[0]:
                    tones_list = None
            except Exception as e:
                rospy.logwarn("make_tone failed: %s", e)
                tones_list = None

        if tones_list is None:
            # 簡易フォールバック
            freqs = EMO_TONE.get(emo, EMO_TONE["interest"])
            tones_list = [{"freq": int(f), "duration": int(DUR_MS)} for f in freqs]

        self.pub_beep.publish(json.dumps(tones_list, ensure_ascii=False))


def main():
    ExpressionHub()
    rospy.spin()

if __name__ == "__main__":
    main()


