#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Dict, Any


VALID_ACTIONS = {
    "observe",
    "invite",
    "paint",
    "show",
    "eat",
    "idle",
    "praise",
    "pet",
    "observe_human_show",
    "sulk",
}


def _max_desire_label(desire: Dict[str, float]) -> str:
    scores = {
        "invite": float(desire.get("want_with_people", 0.0)),
        "paint": float(desire.get("want_paint", 0.0)),
        "show": float(desire.get("want_show", 0.0)),
        "eat": float(desire.get("want_eat", 0.0)),
        "idle": float(desire.get("want_idle", 0.0)),
        "observe": 0.1,
    }
    return max(scores, key=scores.get)


def propose_next_action(inputs: Dict[str, Any]) -> Dict[str, Any]:
    system_started = bool(inputs.get("system_started", True))
    seconds_since_start = float(inputs.get("seconds_since_start", 999.0))
    start_grace_sec = float(inputs.get("start_grace_sec", 10.0))

    allow_paint = bool(inputs.get("allow_paint", False))
    last_policy = inputs.get("last_policy")
    sulk_active = bool(inputs.get("sulk_active", False))

    recent_human_events = inputs.get("recent_human_events", {}) or {}
    desire = inputs.get("desire", {}) or {}
    cooldowns = inputs.get("cooldowns", {}) or {}
    recent_exec = inputs.get("recent_exec", {}) or {}
    min_state_interval_sec = float(inputs.get("min_state_interval_sec", 120.0))

    debug = {
        "system_started": system_started,
        "seconds_since_start": seconds_since_start,
        "start_grace_sec": start_grace_sec,
        "allow_paint": allow_paint,
        "last_policy": last_policy,
        "sulk_active": sulk_active,
        "recent_human_events": recent_human_events,
        "desire": desire,
        "cooldowns": cooldowns,
        "recent_exec": recent_exec,
        "min_state_interval_sec": min_state_interval_sec,
    }

    if not system_started:
        return {
            "next_action": "idle",
            "reason": "system_started が false のため",
            "debug": debug,
        }

    # human input 優先
    if recent_human_events.get("praise", False):
        return {
            "next_action": "praise",
            "reason": "直近の human praise を最優先するため",
            "debug": debug,
        }

    if recent_human_events.get("pet", False):
        return {
            "next_action": "pet",
            "reason": "直近の human pet を最優先するため",
            "debug": debug,
        }

    if recent_human_events.get("offer_food", False):
        return {
            "next_action": "eat",
            "reason": "直近の food offer を最優先するため",
            "debug": debug,
        }

    if recent_human_events.get("show_art", False):
        return {
            "next_action": "observe_human_show",
            "reason": "直近の show_art を最優先するため",
            "debug": debug,
        }

    # 起動直後猶予
    if seconds_since_start < start_grace_sec:
        return {
            "next_action": "idle",
            "reason": "起動直後の grace period 内のため",
            "debug": debug,
        }

    # SULK 優先
    if sulk_active:
        return {
            "next_action": "sulk",
            "reason": "sulk_active が true のため",
            "debug": debug,
        }

    # policy override or desire
    if isinstance(last_policy, str) and last_policy in VALID_ACTIONS:
        choice = last_policy
        reason = f"policy override により {last_policy} が指定されているため"
    else:
        choice = _max_desire_label(desire)
        reason = f"最大欲求ラベルが {choice} のため"

    # invite cooldown
    if choice == "invite" and cooldowns.get("invite_cooldown_active", False):
        return {
            "next_action": "idle",
            "reason": "invite を選びたいが cooldown 中のため idle にフォールバック",
            "debug": {**debug, "preliminary_choice": choice},
        }

    # paint gate
    if choice == "paint" and not allow_paint:
        return {
            "next_action": "idle",
            "reason": "paint を選びたいが allow_paint=false のため idle にフォールバック",
            "debug": {**debug, "preliminary_choice": choice},
        }

    # same-state re-entry suppression
    tracked = {"invite", "paint", "show", "eat", "sulk"}
    if choice in tracked:
        sec_since_last = float(recent_exec.get(choice, 9999.0))
        if sec_since_last < min_state_interval_sec:
            return {
                "next_action": "idle",
                "reason": f"{choice} は直近 {sec_since_last:.1f}s 前に実行済みで、再突入抑制により idle にフォールバック",
                "debug": {**debug, "preliminary_choice": choice},
            }

    return {
        "next_action": choice,
        "reason": reason,
        "debug": debug,
    }


def build_mock_trace_event(
    step_index: int,
    scene: Dict[str, Any],
    decision: Dict[str, Any],
    character_name: str = "Jedy",
) -> Dict[str, Any]:
    action = decision["next_action"]

    default_text = {
        "observe_human_show": "みせてくれてありがとう。",
        "invite": "ぼくもいっしょに描きたいな。",
        "paint": "よし、描いてみるね。",
        "show": "できたよ。見てみて！",
        "praise": "ありがとう！",
        "pet": "なでなで、うれしいな。",
        "eat": "いただきます、って気分だな。",
        "sulk": "ちょっとしょんぼり…。",
        "idle": "今は少しようすを見ているよ。",
        "observe": "見ているよ。",
    }

    emotion_map = {
        "observe_human_show": "interest",
        "invite": "interest",
        "paint": "focus",
        "show": "joy",
        "praise": "joy",
        "pet": "calm",
        "eat": "calm",
        "sulk": "sad",
        "idle": "neutral",
        "observe": "neutral",
    }

    return {
        "step_index": step_index,
        "character_name": character_name,
        "image_filename": scene["image_filename"],
        "human_event": scene.get("human_event", "none"),
        "selected_object": scene.get("selected_object", "unknown"),
        "selected_action": action,
        "emotion": emotion_map.get(action, "neutral"),
        "robot_text": default_text.get(action, ""),
        "reason": decision["reason"],
        "mode": "event",
        "robot_state": scene.get("robot_state", {}),
        "human_state": scene.get("human_state", {}),
        "extra_info": scene.get("extra_info", {}),
    }
