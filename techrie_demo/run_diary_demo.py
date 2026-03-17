#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import argparse
from pathlib import Path

from config import resolve_paths_full
import main_pipeline


SAMPLE_SCENES = [
    {
        "image_filename": "scene1.jpg",
        "selected_object": "moon piece",
        "selected_action": "human_show_art",
        "emotion": "joy",
        "mode": "event",
        "robot_state": {
            "interest": 0.85,
            "fatigue": 0.20,
            "last_action": "observe_human_show",
            "mode": "paint",
            "sociality_score": 0.75
        },
        "human_state": {
            "emotion": "happy",
            "action": "showing drawing",
            "needs": "share",
            "interaction": "close",
            "gaze_target": "robot"
        },
        "extra_info": {
            "weather": "sunny",
            "temperature": "23C"
        }
    },
    {
        "image_filename": "scene2.jpg",
        "selected_object": "drawing",
        "selected_action": "invite",
        "emotion": "interest",
        "mode": "event",
        "robot_state": {
            "interest": 0.92,
            "fatigue": 0.30,
            "last_action": "invite",
            "mode": "paint",
            "sociality_score": 0.82
        },
        "human_state": {
            "emotion": "excited",
            "action": "accepting invitation",
            "needs": "create",
            "interaction": "collaborative",
            "gaze_target": "robot"
        },
        "extra_info": {
            "weather": "sunny",
            "temperature": "23C"
        }
    },
    {
        "image_filename": "scene3.jpg",
        "selected_object": "colorful paper",
        "selected_action": "paint",
        "emotion": "wow",
        "mode": "event",
        "robot_state": {
            "interest": 0.88,
            "fatigue": 0.35,
            "last_action": "paint",
            "mode": "show",
            "sociality_score": 0.80
        },
        "human_state": {
            "emotion": "surprised",
            "action": "watching",
            "needs": "understand",
            "interaction": "engaged",
            "gaze_target": "robot"
        },
        "extra_info": {
            "weather": "sunny",
            "temperature": "23C"
        }
    },
    {
        "image_filename": "scene4.jpg",
        "selected_object": "finished picture",
        "selected_action": "show",
        "emotion": "joy",
        "mode": "event",
        "robot_state": {
            "interest": 0.77,
            "fatigue": 0.55,
            "last_action": "show",
            "mode": "finish",
            "sociality_score": 0.78
        },
        "human_state": {
            "emotion": "proud",
            "action": "showing result",
            "needs": "be_seen",
            "interaction": "warm",
            "gaze_target": "robot"
        },
        "extra_info": {
            "weather": "sunny",
            "temperature": "23C"
        }
    }
]


def ensure_dir(path_str: str):
    Path(path_str).mkdir(parents=True, exist_ok=True)


def write_json_if_missing(path_str: str, obj):
    p = Path(path_str)
    if p.exists() and p.stat().st_size > 0:
        print(f"[skip] already exists: {p}")
        return
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    print(f"[init] created: {p}")


def init_sample_inputs(date_str: str):
    paths = resolve_paths_full(date_str)
    ensure_dir(paths["day_dir"])
    write_json_if_missing(paths["scenes_json"], SAMPLE_SCENES)

    print("\nPlace image files with these names in:")
    for scene in SAMPLE_SCENES:
        print(f"  {Path(paths['day_dir']) / scene['image_filename']}")
    print()

    return paths


def validate_inputs(paths: dict):
    missing = []

    scenes_json = Path(paths["scenes_json"])
    if not scenes_json.exists():
        missing.append(f"missing scenes_json: {scenes_json}")
        return missing

    with scenes_json.open("r", encoding="utf-8") as f:
        scenes = json.load(f)

    if not isinstance(scenes, list) or len(scenes) == 0:
        missing.append(f"scenes_json is empty or not a list: {scenes_json}")
        return missing

    for i, entry in enumerate(scenes):
        img = entry.get("image_filename")
        if not img:
            missing.append(f"scene[{i}] has no image_filename")
            continue
        img_path = Path(paths["day_dir"]) / img
        if not img_path.exists():
            missing.append(f"missing image for scene[{i}]: {img_path}")

    return missing


def print_paths(paths: dict):
    print("Resolved paths:")
    for k, v in paths.items():
        print(f"  {k}: {v}")
    print()


def main():
    parser = argparse.ArgumentParser(description="Offline diary-generation demo runner.")
    parser.add_argument("--date", required=True, help="e.g. 2026-03-16")
    parser.add_argument("--init-sample", action="store_true", help="create sample scenes_json")
    parser.add_argument("--dry-run", action="store_true", help="show paths and validate only")
    parser.add_argument("--skip-image-gen", action="store_true")
    parser.add_argument("--skip-merge", action="store_true")
    parser.add_argument("--skip-profile-reflect", action="store_true")
    args = parser.parse_args()

    paths = resolve_paths_full(args.date)

    if args.init_sample:
        paths = init_sample_inputs(args.date)

    print_paths(paths)

    problems = validate_inputs(paths)
    if problems:
        print("Input validation failed:")
        for p in problems:
            print(f"  - {p}")
        print("\nFix the above issues, then rerun.")
        return

    if args.dry_run:
        print("[dry-run] validation passed.")
        return

    print(f"[run] starting diary pipeline for {args.date}")
    main_pipeline.main(
        args.date,
        skip_image_gen=args.skip_image_gen,
        skip_merge=args.skip_merge,
        skip_profile_reflect=args.skip_profile_reflect,
    )
    print("[done] diary pipeline finished.")

    print("\nExpected outputs:")
    for key in ["captioned_json", "diary_json", "diary_image", "diary_combined"]:
        print(f"  {key}: {paths[key]}")


if __name__ == "__main__":
    main()
