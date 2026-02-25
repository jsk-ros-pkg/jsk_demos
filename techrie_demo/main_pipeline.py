# main_pipeline.py
from step1_select_scene import select_scenes
from step2_generate_caption import generate_captions
from step3_generate_diary import generate_diary_text
from step4_generate_image import generate_diary_image
from step5_merge_image_and_text import merge_image_and_text
from gpt_profile_reflect import run_gpt_profile_reflect

from config import resolve_paths_full
from rospkg import RosPack
from pathlib import Path
from datetime import datetime
import os, json
import datetime as dt

def _safe_load_json(path):
    p = Path(path)
    if not p.exists() or p.stat().st_size == 0:
        return None
    try:
        with p.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None

def _valid_diary(obj):
    return isinstance(obj, dict) and "text" in obj and isinstance(obj["text"], str) and obj["text"].strip()

def _valid_captioned(obj):
    # ざっくり：リストで、要素がdict
    return isinstance(obj, list) and (len(obj) == 0 or isinstance(obj[0], dict))

def _load_profile():
    # techrie_demo/config/Jedy.json を読む
    pkg = RosPack().get_path("techrie_demo")
    prof_path = os.path.join(pkg, "config", "Jedy.json")
    with open(prof_path, "r", encoding="utf-8") as f:
        return json.load(f)

def _load_raw_scenes(paths):
    # scenes_json が無ければ空の配列で作る
    sj = Path(paths["scenes_json"])
    sj.parent.mkdir(parents=True, exist_ok=True)
    if not sj.exists():
        sj.write_text("[]", encoding="utf-8")
    with open(sj, "r", encoding="utf-8") as f:
        return json.load(f)

def main(date_str=None):

    # ここで date_str を確定
    if date_str is None:
        date_str = dt.date.today().isoformat()
    
    # パスを辞書で取得（config.py の resolve_paths_full を使用）
    paths = resolve_paths_full(date_str)
    
    # ここで必ず date_str を辞書に入れる（←これが今回の修正ポイント）
    paths["date_str"] = date_str


    # 旧コード互換のキー名も追加しておく（step* が参照する可能性があるため）
    paths["json_path"]           = paths["scenes_json"]
    paths["captioned_path"]     = paths["captioned_json"]
    paths["diary_output_path"]  = paths["diary_json"]
    paths["output_image_path"]  = paths["diary_image"]
    paths["combined_output_path"]= paths["diary_combined"]

    # プロフィールと生データをロード
    profile  = _load_profile()
    raw_data = _load_raw_scenes(paths)

    if not raw_data:
        raw_data = [{"image_filename": None, "selected_object": "unknown",
                     "selected_action": "note", "emotion": "neutral"}]

    # Step 1: 印象的な場面を選出 → キャプション生成
    if Path(paths["captioned_json"]).exists():
        print("✔ captioned.json が既に存在します。スキップします。")
        with open(paths["captioned_json"], "r", encoding="utf-8") as f:
            raw_data = json.load(f)
    else:
        selected_indices = select_scenes(profile, raw_data)
        raw_data = generate_captions(profile, raw_data, selected_indices, paths)

    # Step 2 & 3: 絵日記テキスト生成
    if Path(paths["diary_json"]).exists():
        print("✔ diary.json が既に存在します。スキップします。")
        with open(paths["diary_json"], "r", encoding="utf-8") as f:
            diary_text = json.load(f)["text"]
    else:
        diary_text = generate_diary_text(profile, raw_data, paths)

    # Step 4: 絵日記画像生成
    if Path(paths["diary_image"]).exists():
        print("✔ 絵日記画像が既に存在します。スキップします。")
    else:
        generate_diary_image(diary_text, paths, profile)

    # Step 5: イラスト＋テキストを合成した絵日記画像を作成
    merge_image_and_text(paths, diary_text)

    # Step 6: プロフィール更新（毎週日曜21時以降のみ実行）
    if datetime.now().weekday() == 6 and datetime.now().hour >= 21:
        run_gpt_profile_reflect()
    else:
        print("🕒 日曜21時以降ではないため、プロフィール更新はスキップされました。")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--date", type=str, help="日付 (例: 2025-05-06)")
    args = parser.parse_args()
    main(args.date)


