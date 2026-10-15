#!/usr/bin/env python3
import argparse
import csv
import os
import wave
import rosbag
from datetime import datetime

# ===== TSVとして出力するトピック =====
TEXT_TOPICS = [
    "/kashiwagi_state",
    "/speech_to_text",
    "/talking_game_response",
]

# ===== WAVとして出力するトピック =====
AUDIO_TOPIC = "/audio"

# ===== audio設定 =====
# /audio の中身が 16bit PCM / 16000Hz / mono の場合
AUDIO_SAMPLE_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_SAMPLE_WIDTH = 2  # bytes。16bit PCMなら2、8bitなら1

OUTPUT_FORMAT = "tsv"


def format_timestamp(t):
    return datetime.fromtimestamp(t.to_sec()).strftime("%Y-%m-%d %H:%M:%S")


def extract_data(topic, msg):
    if topic == "/kashiwagi_state":
        return msg.data

    elif topic == "/speech_to_text":
        # transcript が空の可能性に備える
        if hasattr(msg, "transcript") and len(msg.transcript) > 0:
            return msg.transcript[0]
        return ""

    elif topic == "/talking_game_response":
        return msg.data

    else:
        return str(msg)


def topic_to_suffix(topic):
    # "/speech_to_text" -> "speech_to_text"
    return topic.strip("/").replace("/", "_")


def make_tsv_output_path(bag_path, topic):
    # "/path/to/input.bag" -> "/path/to/input_speech_to_text.tsv"
    bag_dir = os.path.dirname(bag_path)
    bag_base = os.path.splitext(os.path.basename(bag_path))[0]
    topic_name = topic_to_suffix(topic)

    return os.path.join(bag_dir, f"{bag_base}_{topic_name}.tsv")


def make_wav_output_path(bag_path, topic):
    # "/path/to/input.bag" -> "/path/to/input_audio.wav"
    bag_dir = os.path.dirname(bag_path)
    bag_base = os.path.splitext(os.path.basename(bag_path))[0]
    topic_name = topic_to_suffix(topic)

    return os.path.join(bag_dir, f"{bag_base}_{topic_name}.wav")


def write_audio_msg_to_wav(wav_file, msg):
    """
    audio_common_msgs/AudioData のように msg.data が uint8[] の前提。
    """
    if not hasattr(msg, "data"):
        raise AttributeError(f"{AUDIO_TOPIC} のメッセージに data フィールドがありません")

    wav_file.writeframes(bytes(msg.data))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="input bag file path")
    args = parser.parse_args()

    bag_path = args.bag
    delimiter = "\t"

    files = {}
    writers = {}

    wav_path = make_wav_output_path(bag_path, AUDIO_TOPIC)

    try:
        # TSVファイルをトピックごとに開く
        for topic in TEXT_TOPICS:
            output_path = make_tsv_output_path(bag_path, topic)

            f = open(output_path, "w", newline="")
            writer = csv.writer(f, delimiter=delimiter)
            writer.writerow(["timestamp", "data"])

            files[topic] = f
            writers[topic] = writer

            print(f"TSV output: {output_path}")

        # WAVファイルを開く
        wav_file = wave.open(wav_path, "wb")
        wav_file.setnchannels(AUDIO_CHANNELS)
        wav_file.setsampwidth(AUDIO_SAMPLE_WIDTH)
        wav_file.setframerate(AUDIO_SAMPLE_RATE)

        print(f"WAV output: {wav_path}")

        # bagを読む
        topics_to_read = TEXT_TOPICS + [AUDIO_TOPIC]

        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=topics_to_read):

                if topic in TEXT_TOPICS:
                    writers[topic].writerow([
                        format_timestamp(t),
                        extract_data(topic, msg),
                    ])

                elif topic == AUDIO_TOPIC:
                    write_audio_msg_to_wav(wav_file, msg)

    finally:
        for f in files.values():
            f.close()

        try:
            wav_file.close()
        except NameError:
            pass


if __name__ == "__main__":
    main()
