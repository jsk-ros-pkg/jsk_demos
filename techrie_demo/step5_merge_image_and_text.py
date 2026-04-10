# step5_merge_image_and_text.py
from PIL import Image, ImageDraw, ImageFont
from pathlib import Path
import textwrap


def merge_image_and_text(paths, diary_text):
    image_path = Path(paths["output_image_path"])
    output_path = image_path.parent / "diary_combined.png"

    if output_path.exists():
        print("✔ 合成画像が既に存在します。スキップします。")
        return

    # 読み込み
    image = Image.open(image_path).convert("RGB")

    # フォント設定（環境に応じて調整）
    # フォント設定（日本語対応フォントに変更）
    font_path_candidates = [
        "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
        "/Library/Fonts/Arial Unicode.ttf"
    ]
    for path in font_path_candidates:
        if Path(path).exists():
            font = ImageFont.truetype(path, 28)
            break
    else:
        print("⚠️ 日本語フォントが見つかりません。デフォルトフォントを使用します。")
        font = ImageFont.load_default()


    # テキスト整形
    wrapped_text = textwrap.fill(diary_text, width=35)

    # ダミー画像で描画領域の bbox を取得
    dummy_img = Image.new("RGB", (image.width, 1000), "white")
    dummy_draw = ImageDraw.Draw(dummy_img)
    text_bbox = dummy_draw.textbbox((30, 0), wrapped_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1] + 60  # 上下余白込み

    # テキストを中央に描画する位置
    text_x = (image.width - text_width) // 2
    text_y = image.height + 30

    # 新しい画像（上：元画像、下：テキスト）
    combined = Image.new("RGB", (image.width, image.height + text_height), "white")
    combined.paste(image, (0, 0))

    # テキスト描画
    draw = ImageDraw.Draw(combined)
    draw.text((text_x, text_y), wrapped_text, fill="black", font=font)

    # 保存
    combined.save(output_path)
    print(f"🖼️ 合成絵日記を保存: {output_path}")
