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
import os
import json
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


def _load_profile():
    pkg = RosPack().get_path("techrie_demo")
    prof_path = os.path.join(pkg, "config", "Jedy.json")
    with open(prof_path, "r", encoding="utf-8") as f:
        return json.load(f)


def _load_raw_scenes(paths):
    sj = Path(paths["scenes_json"])
    sj.parent.mkdir(parents=True, exist_ok=True)
    if not sj.exists():
        sj.write_text("[]", encoding="utf-8")
    with open(sj, "r", encoding="utf-8") as f:
        return json.load(f)


def main(
    date_str=None,
    skip_image_gen=False,
    skip_merge=False,
    skip_profile_reflect=False,
):
    if date_str is None:
        date_str = dt.date.today().isoformat()

    paths = resolve_paths_full(date_str)
    paths["date_str"] = date_str

    # 旧コード互換キー
    paths["json_path"] = paths["scenes_json"]
    paths["captioned_path"] = paths["captioned_json"]
    paths["diary_output_path"] = paths["diary_json"]
    paths["output_image_path"] = paths["diary_image"]
    paths["combined_output_path"] = paths["diary_combined"]

    profile = _load_profile()
    raw_data = _load_raw_scenes(paths)

    if not raw_data:
        raw_data = [{
            "image_filename": None,
            "selected_object": "unknown",
            "selected_action": "note",
            "emotion": "neutral"
        }]

    # Step 1 & 2
    if Path(paths["captioned_json"]).exists():
        print("✔ captioned.json が既に存在します。スキップします。")
        with open(paths["captioned_json"], "r", encoding="utf-8") as f:
            raw_data = json.load(f)
    else:
        selected_indices = select_scenes(profile, raw_data)
        raw_data = generate_captions(profile, raw_data, selected_indices, paths)

    # Step 3
    if Path(paths["diary_json"]).exists():
        print("✔ diary.json が既に存在します。スキップします。")
        with open(paths["diary_json"], "r", encoding="utf-8") as f:
            diary_text = json.load(f)["text"]
    else:
        diary_text = generate_diary_text(profile, raw_data, paths)

    # Step 4
    if skip_image_gen:
        print("⏭ skip_image_gen=True のため、絵日記画像生成をスキップします。")
    else:
        if Path(paths["diary_image"]).exists():
            print("✔ 絵日記画像が既に存在します。スキップします。")
        else:
            generate_diary_image(diary_text, paths, profile)

    # Step 5
    if skip_merge:
        print("⏭ skip_merge=True のため、画像とテキストの合成をスキップします。")
    else:
        if Path(paths["diary_image"]).exists():
            merge_image_and_text(paths, diary_text)
        else:
            print("⏭ diary_image が存在しないため、合成をスキップします。")

    # Step 6
    if skip_profile_reflect:
        print("⏭ skip_profile_reflect=True のため、プロフィール更新をスキップします。")
    else:
        if datetime.now().weekday() == 6 and datetime.now().hour >= 21:
            run_gpt_profile_reflect()
        else:
            print("🕒 日曜21時以降ではないため、プロフィール更新はスキップされました。")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--date", type=str, help="日付 (例: 2026-03-16)")
    parser.add_argument("--skip-image-gen", action="store_true")
    parser.add_argument("--skip-merge", action="store_true")
    parser.add_argument("--skip-profile-reflect", action="store_true")
    args = parser.parse_args()

    main(
        args.date,
        skip_image_gen=args.skip_image_gen,
        skip_merge=args.skip_merge,
        skip_profile_reflect=args.skip_profile_reflect,
    )
