#!/usr/bin/env python3.8
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Float32
from scipy.signal import lfilter

# シンプルなローパスフィルタを定義
def low_pass_filter(data, alpha=0.1):
    return lfilter([1-alpha], [1, -alpha], data)

# RMSを計算するための関数
def calculate_rms(audio_data):
    # ローパスフィルタを適用してノイズを軽減
    filtered_data = low_pass_filter(audio_data)
    rms = np.sqrt(np.mean(np.square(filtered_data)))
    return rms

def audio_callback(msg):
    # バイナリデータをnumpy配列に変換
    audio_data = np.frombuffer(msg.data, dtype=np.int16)
    
    # RMS（音量）を計算
    rms = calculate_rms(audio_data)
    
    # デシベル (dB) に変換
    volume_db = 20 * np.log10(rms) if rms > 0 else 0.0
    
    # 音量をパブリッシュ
    volume_publisher.publish(volume_db)

def listener():
    rospy.init_node('audio_volume_calculator')
    
    # 音量をパブリッシュするためのパブリッシャを作成
    global volume_publisher
    volume_publisher = rospy.Publisher('/audio_volume', Float32, queue_size=10)
    
    # /audioトピックを購読
    rospy.Subscriber("/audio", AudioData, audio_callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
