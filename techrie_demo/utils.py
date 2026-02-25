# utils.py
import base64
from pathlib import Path

def encode_image_to_base64(image_path: Path) -> str:
    """指定された画像ファイルをbase64エンコードして返す"""
    with open(image_path, "rb") as img_f:
        return base64.b64encode(img_f.read()).decode("utf-8")
