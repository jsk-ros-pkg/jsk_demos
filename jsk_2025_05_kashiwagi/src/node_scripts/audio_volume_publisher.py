#!/usr/bin/env python
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Float32
from scipy.signal import lfilter
import time

# シンプルなローパスフィルタ
def low_pass_filter(data, alpha=0.1):
    return lfilter([1 - alpha], [1, -alpha], data)

# RMSを計算
def calculate_rms(audio_data):
    filtered_data = low_pass_filter(audio_data)
    return np.sqrt(np.mean(np.square(filtered_data)))

# グローバル変数
last_process_time = 0.0
process_interval = 0.1  # 秒（0.1 秒おきに処理）

def audio_callback(msg):
    global last_process_time

    now = time.time()
    if now - last_process_time < process_interval:
        return  # 前回から0.1秒未満ならスキップ
    last_process_time = now

    # バイナリデータをnumpy配列に変換
    audio_data = np.frombuffer(msg.data, dtype=np.int16)

    # RMS（音量）を計算
    rms = calculate_rms(audio_data)

    # デシベル (dB) に変換
    volume_db = 20 * np.log10(rms) if rms > 0 else 0.0

    # 音量をパブリッシュ
    volume_publisher.publish(volume_db)
    rospy.loginfo_throttle(0.5, f"Volume: {volume_db:.2f} dB")

def listener():
    rospy.init_node('audio_volume_calculator')
    rospy.sleep(1.0)

    global volume_publisher
    volume_publisher = rospy.Publisher('/audio_volume', Float32, queue_size=10)

    # 最新データのみ処理するように queue_size=1
    rospy.Subscriber("/audio", AudioData, audio_callback, queue_size=1, buff_size=2**20)

    rospy.loginfo("Launching audio volume calculator node ...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
