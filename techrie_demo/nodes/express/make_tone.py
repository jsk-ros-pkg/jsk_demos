import os
import json
from pathlib import Path

import openai
import rospy
from std_msgs.msg import String

def _rhythm_scale():
    # import時ではなく呼び出し時に読む
    try:
        return float(rospy.get_param("/tone_rhythm_scale", 0.5))
    except Exception:
        return 0.5

def _load_api_cfg():
    # まず env を優先
    api_key = os.getenv("TECHRIE_TONE_API_KEY") or os.getenv("TECHRIE_TEXT_API_KEY")
    base_url = os.getenv("TECHRIE_TONE_BASE_URL") or os.getenv("TECHRIE_TEXT_BASE_URL")
    api_version = os.getenv("TECHRIE_TONE_API_VERSION") or os.getenv("TECHRIE_TEXT_API_VERSION") or "2024-12-01-preview"

    # envがなければ config/gpt_api.yaml を読む
    if not (api_key and base_url):
        try:
            import yaml
            # nodes/express/make_tone.py -> package root は2階層上
            pkg_root = Path(__file__).resolve().parents[2]
            cfg_path = pkg_root / "config" / "gpt_api.yaml"
            if cfg_path.exists():
                y = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
                text = y.get("text", {})
                api_key = api_key or text.get("api_key")
                base_url = base_url or text.get("base_url")
                api_version = api_version or text.get("api_version")
        except Exception:
            pass

    if not api_key or not base_url:
        raise RuntimeError("Tone Azure OpenAI config is missing. Set TECHRIE_TEXT_* or config/gpt_api.yaml")

    return api_key, api_version, base_url

_client = None
def _get_client():
    global _client
    if _client is None:
        api_key, api_version, base_url = _load_api_cfg()
        _client = openai.AzureOpenAI(
            api_key=api_key,
            api_version=api_version,
            base_url=base_url,
        )
    return _client


def generate_tone_sequence(word):
    """GPT-4o を使って単語から音階リストを生成する"""
    prompt = f"""
    Generate a sequence of musical notes that sound like the word '{word}'. 
    The output should be in JSON format with a list of dictionaries, 
    where each dictionary contains:
    - "freq": frequency in Hz (e.g., 440 for A4)
    - "duration": duration in milliseconds (e.g., 500)
    Example output:
    [
        {{"freq": 440, "duration": 500}},
        {{"freq": 660, "duration": 300}},
        {{"freq": 550, "duration": 400}}
    ]
    You must ommit the explanation and output only json.
    """
    response = _get_client().chat.completions.create(...)

    try:
        tones = json.loads(response.choices[0].message.content)
        return tones
    except json.JSONDecodeError:
        return [{"freq": 440, "duration": 500}]  # デフォルトの音を返す

import re

C_MAJOR_SCALE = [261, 293, 329, 349, 392, 440, 493, 523]

RHYTHM_PATTERNS = {
    "joy":      [200, 150, 300, 150],
    "interest": [250, 200, 250, 300],
    "anger":    [150, 150, 150, 150],
    "bore":     [400, 600],
    "sad":      [600, 500],
    "surprise": [100, 400, 100],
    "fear":     [100, 200, 100, 400],
    "trust":    [300, 200, 300],
    "neutral":  [300, 300, 300],
}




def generate_tone_sequence_simple(word):
    """単語の文字列から音階リストを生成する（GPT不使用）"""
    tones = []
    word = word.lower()
    MAX_TONES = 10

    for i, char in enumerate(word[:MAX_TONES]):
        note_index = (ord(char) + i) % len(C_MAJOR_SCALE)
        freq = C_MAJOR_SCALE[note_index]
        duration = 80  # 固定長 or ランダムで変更可能
        tones.append({"freq": freq, "duration": duration})

    return tones


def generate_tone_sequence_by_emotion(word, emotion="neutral"):
    rhythm = RHYTHM_PATTERNS.get(emotion, RHYTHM_PATTERNS["neutral"])
    tones = []
    word = word.lower()
    MAX_TONES = 20

    for i, char in enumerate(word[:MAX_TONES]):
        note_index = (ord(char) + i) % len(C_MAJOR_SCALE)
        freq = C_MAJOR_SCALE[note_index]
        duration = rhythm[i % len(rhythm)]
        duration = int(rhythm[i % len(rhythm)] * _rhythm_scale())
        tones.append({"freq": freq, "duration": duration})

    return tones



def send_tone_sequence(word,emotion):
    """ROSトピックに音階リストを送信する"""
    rospy.init_node('gpt4o_tone_publisher', anonymous=True)
    pub = rospy.Publisher('tone_sequence', String, queue_size=10)
    tones = generate_tone_sequence_by_emotion(word,emotion)
    tones_json = json.dumps(tones)  # JSON形式に変換
    rospy.loginfo(f"Sending: {tones_json}")
    
    pub.publish(tones_json)

if __name__ == "__main__":
    word = input("Enter a word: ")  # ユーザーから単語を入力
    emotion = "neutral"
    send_tone_sequence(word,emotion)

# === ここから追記 ===
import math

NOTE_FREQS = {
    # 12-TET, A4=440
    "C4":261.63,"D4":293.66,"E4":329.63,"F4":349.23,"G4":392.00,"A4":440.00,"B4":493.88,
    "C5":523.25,"D5":587.33,"E5":659.25,"F5":698.46,"G5":783.99,"A5":880.00,"B5":987.77,
}

def _beats_to_ms(beats, bpm):
    return int(60000.0 / bpm * beats * _rhythm_scale())

def generate_sing_sequence(notes, durations, bpm=120, vibrato_hz=5.0, vibrato_cents=20.0):
    """
    notes: 例 ["C4","C4","G4","G4","A4","A4","G4", ...]
    durations: 各音符の拍長 [1,1,1,1,1,1,2, ...]（4分=1拍想定）
    bpm: テンポ
    vibrato_cents: ±何セント揺らすか（20c=わずか）
    """
    assert len(notes) == len(durations)
    frame_ms = 20  # ここを10～25の範囲で調整可（短いほど滑らか＆CPU増）
    out = []
    for note, beat in zip(notes, durations):
        base = NOTE_FREQS[note]
        total = _beats_to_ms(beat, bpm)
        t = 0
        while t < total:
            # sinで±セント揺らす
            vib_ratio = (vibrato_cents/1200.0) * math.sin(2*math.pi*vibrato_hz*(t/1000.0))
            f = base * (2.0 ** vib_ratio)
            out.append({"freq": int(round(f)), "duration": frame_ms})
            t += frame_ms
    return out

def generate_twinkle(bpm=100):
    # きらきら星（前半）/ 拍は 4分=1 8分=0.5 2分=2 のように
    notes = ["C4","C4","G4","G4","A4","A4","G4",
             "F4","F4","E4","E4","D4","D4","C4"]
    durs  = [1,1,1,1,1,1,2,
             1,1,1,1,1,1,2]
    return generate_sing_sequence(notes, durs, bpm=bpm, vibrato_hz=5.0, vibrato_cents=18.0)

def send_song_twinkle(bpm=100):
    rospy.init_node('sing_twinkle_publisher', anonymous=True)
    pub = rospy.Publisher('tone_sequence', String, queue_size=10)
    tones = generate_twinkle(bpm=bpm)
    pub.publish(json.dumps(tones, ensure_ascii=False))
    rospy.loginfo("Twinkle sent.")
# === 追記ここまで ===
