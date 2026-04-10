# step2_generate_caption.py
from pathlib import Path
from config import client
from utils import encode_image_to_base64
import json


def generate_captions(profile, raw_data, indices, paths):
    for i in indices:
        entry = raw_data[i]
        image_path = Path(paths["json_path"]).parent / entry["image_filename"]
        if not image_path.exists():
            print(f"⚠️ 画像が見つかりません: {image_path}")
            continue

        base64_image = encode_image_to_base64(image_path)
        prompt_caption = f"""
        これはロボット「{profile['name']}」が撮った写真です。
        短く、子ども向けの1文のキャプションを作ってください。

        - 注目したもの: {entry.get('selected_object', '不明')}
        - 行動: {entry.get('selected_action', '不明')}
        - 感情: {entry.get('emotion', '不明')}
        """

        caption_response = client.chat.completions.create(
            model="o1",
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt_caption},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                ]
            }],
        )

        caption = caption_response.choices[0].message.content.strip()
        raw_data[i]["caption"] = caption
        print(f"✔ {entry['image_filename']} にキャプション追加: {caption}")

    with open(paths["captioned_path"], "w", encoding="utf-8") as f:
        json.dump(raw_data, f, ensure_ascii=False, indent=2)

    return raw_data
