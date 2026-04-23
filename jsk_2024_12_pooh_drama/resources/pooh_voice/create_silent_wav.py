import numpy as np
from scipy.io.wavfile import write

# パラメータ設定
sampling_rate = 44100  # サンプリングレート（44.1kHz）
duration = 3           # 長さ（秒）
amplitude = 0          # 音の振幅（0 = 無音）

# 無音データを生成
silent_data = np.zeros(int(sampling_rate * duration), dtype=np.int16)

# WAVファイルとして保存
write("silent_3sec.wav", sampling_rate, silent_data)

print("無音のWAVファイルを作成しました: silent_3sec.wav")
