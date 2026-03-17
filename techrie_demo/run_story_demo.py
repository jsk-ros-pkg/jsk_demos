#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import argparse
from pathlib import Path

from config import resolve_paths_full
import main_pipeline
from action_policy import propose_next_action, build_mock_trace_event


def load_story(path_str: str):
    with open(path_str, "r", encoding="utf-8") as f:
        obj = json.load(f)
    if not isinstance(obj, dict):
        raise ValueError("story json must be a dict")
    if "scenes" not in obj or not isinstance(obj["scenes"], list):
        raise ValueError("story json must contain a list field named 'scenes'")
    return obj


def ensure_day_dir(paths: dict):
    Path(paths["day_dir"]).mkdir(parents=True, exist_ok=True)


def save_json(path_str: str, obj):
    p = Path(path_str)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


def print_trace(trace):
    print("\n=== miniature interaction replay ===")
    for ev in trace:
        print(f"[{ev['step_index']}] human_event={ev['human_event']} -> robot_action={ev['selected_action']}")
        if ev["robot_text"]:
            print(f"    {ev['character_name']}: {ev['robot_text']}")
        print(f"    reason: {ev['reason']}")
    print("=== end replay ===\n")


def main():
    parser = argparse.ArgumentParser(description="Miniature event + diary demo runner.")
    parser.add_argument("--date", required=True, help="e.g. 2026-03-16")
    parser.add_argument("--story", required=True, help="path to story json")
    parser.add_argument("--write-scenes-only", action="store_true")
    parser.add_argument("--skip-image-gen", action="store_true")
    parser.add_argument("--skip-merge", action="store_true")
    parser.add_argument("--skip-profile-reflect", action="store_true")
    args = parser.parse_args()

    story = load_story(args.story)
    character_name = story.get("character_name", "Jedy")
    shared_inputs = story.get("shared_inputs", {})

    trace = []
    for i, scene in enumerate(story["scenes"], start=1):
        inputs = {}
        inputs.update(shared_inputs)
        inputs.update(scene.get("decision_inputs", {}))
        decision = propose_next_action(inputs)
        event = build_mock_trace_event(
            step_index=i,
            scene=scene,
            decision=decision,
            character_name=character_name,
        )
        trace.append(event)

    print_trace(trace)

    paths = resolve_paths_full(args.date)
    ensure_day_dir(paths)
    save_json(paths["scenes_json"], trace)
    print(f"[saved] scenes_json: {paths['scenes_json']}")
    print(f"[note] put referenced images into: {paths['day_dir']}")

    if args.write_scenes_only:
        print("[done] write-scenes-only mode")
        return

    main_pipeline.main(
        args.date,
        skip_image_gen=args.skip_image_gen,
        skip_merge=args.skip_merge,
        skip_profile_reflect=args.skip_profile_reflect,
    )

    print("\nExpected outputs:")
    for key in ["captioned_json", "diary_json", "diary_image", "diary_combined"]:
        print(f"  {key}: {paths[key]}")


if __name__ == "__main__":
    main()
