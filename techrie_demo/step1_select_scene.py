# step1_select_scene.py (hybrid ver.)
import re
import json
from typing import List, Dict, Any, Tuple
from config import client

# ====== 可変パラメータ ======
TARGET_NUM_MIN = 4
TARGET_NUM_MAX = 5
TOPK_FOR_GPT   = 12      # ローカル採点の上位KをGPTに渡す
USE_GPT_FINAL  = True    # Falseにすると完全ローカル選抜モード

# アクション優先度（高いほど重要）
ACTION_PRIORITY = {
    "human_pet":        3,
    "human_praise":     3,
    "human_offer_food":     3,
    "paint_begin":      3,
    "paint_done":       3,
    "human_dance":      3,
    "robot_dance_start": 3,
    "self_invite":      2,
    "self_invite_ok":   2,
    "self_invite_ng":   1,
    "robot_present":    2,
    "robot_listen":     2,
    "topic_string":     1,
    "gpt_act":          0,   # デフォルトでは低め
}

# 感情ボーナス（例）
EMO_BONUS = {
    "joy": 2, "want": 1, "wow": 1, "interest": 1,
    "sad": 2, "angry": 1, "fear": 1, "calm": 0, "unknown": 0
}

def _norm(s):
    return (s or "").strip().lower()

def _human_info_bonus(hs: Dict[str, Any]) -> int:
    fields = ["emotion", "action", "needs", "interaction", "gaze_target"]
    known = sum(1 for k in fields if _norm(hs.get(k)) not in ("", "unknown", "不明"))
    # 1項目ごとに+0.5、端数切上げ
    return int((known * 0.5) + 0.5) if known else 0

def _diversity_penalty(action: str, seen_counts: Dict[str, int]) -> int:
    # 同一アクションが多いほどペナルティ
    n = seen_counts.get(action, 0)
    return -n  # 1件目 0, 2件目 -1, 3件目 -2 ...

def _score_entry(entry: Dict[str, Any], seen_counts: Dict[str, int]) -> Tuple[int, Dict[str, Any]]:
    act = _norm(entry.get("selected_action", ""))
    emo = _norm(entry.get("emotion", "unknown"))
    rs  = entry.get("robot_state", {}) or {}
    hs  = entry.get("human_state", {}) or {}

    score = 0
    score += ACTION_PRIORITY.get(act, 0)
    score += EMO_BONUS.get(emo, 0)
    score += _human_info_bonus(hs)
    score += _diversity_penalty(act, seen_counts)

    # 追加の軽微なヒント：robot_state の mode/last_action が有意なら +1
    if _norm(rs.get("last_action")) not in ("", "unknown", "不明"):
        score += 1
    if _norm(rs.get("mode")) not in ("", "unknown", "不明"):
        score += 1

    why = {
        "action": act,
        "emotion": emo,
        "human_info_bonus": _human_info_bonus(hs),
        "diversity_penalty": _diversity_penalty(act, seen_counts),
        "robot_state_signals": 2 if (_norm(rs.get("last_action")) and _norm(rs.get("mode"))) else 1
    }
    return score, why

def _rank_candidates(raw_data: List[Dict[str, Any]]) -> List[Tuple[int, int, Dict[str, Any]]]:
    seen_counts = {}
    ranked = []
    for i, e in enumerate(raw_data):
        act = _norm(e.get("selected_action", ""))
        # まず軽いフィルタ：imageやaction未設定を弾く（必要なら緩めてOK）
        if not e.get("image_filename"):
            continue
        sc, why = _score_entry(e, seen_counts)
        ranked.append((i, sc, why))
        # diversity用カウントは“候補化した時点”で更新
        seen_counts[act] = seen_counts.get(act, 0) + 1

    # スコア降順、同点は「人関与多い/感情ボーナス大/新しい方」を優先するなどのtie-breakerも可
    ranked.sort(key=lambda x: x[1], reverse=True)
    return ranked

def _format_candidate_lines(profile, raw_data, ranked_topk):
    lines = []
    for rank, (idx, sc, why) in enumerate(ranked_topk, 1):
        e = raw_data[idx]
        lines.append(
            f"{rank}. id={idx}, img={e.get('image_filename')}, action={e.get('selected_action')}, "
            f"emotion={e.get('emotion')}, human={e.get('human_state',{})}, score={sc}, why={json.dumps(why, ensure_ascii=False)}"
        )
    return "\n".join(lines)

def _ask_gpt_to_pick(profile, raw_data, ranked_topk):
    # ここでは “4–5件、物語性+多様性+人関与優先” を明文化。JSONで返させる。
    candidates_text = _format_candidate_lines(profile, raw_data, ranked_topk)
    prompt = f"""
あなたは「{profile['name']}」の一日を子ども向け日記にまとめる編集者です。
下の候補から「物語として流れがあり、かつ多様性があり、人との関わりが読み取りやすい」4〜5件を選んでください。

優先ルール（上ほど重要）:
1) 人との関わりがある/読み取れる（human.*にunknownが少ない）
2) アクションの多様性（同じ selected_action ばかりにしない）
3) 感情のコントラスト（joy/want/sad/calm などが混じる）
4) {profile['name']}の一日の流れ（始まり→山→終わり）をイメージ

出力は以下の **厳密なJSON** のみ:
{{
  "indices": [原データのidの数値配列（{TARGET_NUM_MIN}〜{TARGET_NUM_MAX}件）],
  "reason": "短い説明"
}}

候補（ranked top {len(ranked_topk)}。各行の id は元配列の index）:
{candidates_text}
""".strip()

    response = client.chat.completions.create(
        model="o1",
        messages=[{"role": "user", "content": prompt}],
        response_format={"type": "json_object"}  # JSON返答を強制
    )
    content = response.choices[0].message.content
    try:
        obj = json.loads(content)
        ids = obj.get("indices") or []
        # 念のため型/範囲チェック
        ids = [int(i) for i in ids if isinstance(i, int)]
        ids = [i for i in ids if 0 <= i < len(raw_data)]
        if len(ids) < TARGET_NUM_MIN:
            raise ValueError("選出数が少なすぎます")
        return ids
    except Exception as ex:
        # フォールバック：トップから所要数を返す
        return [idx for idx, _, _ in ranked_topk[:TARGET_NUM_MAX]]

def _pick_locally(ranked):
    # 完全ローカル選抜：多様性を確保しつつ上から詰める
    chosen, seen = [], {}
    for idx, sc, why in ranked:
        act = _norm(why.get("action", ""))
        # 多様性：既に同アクションが2件以上なら後回し
        if seen.get(act, 0) >= 2:
            continue
        chosen.append(idx)
        seen[act] = seen.get(act, 0) + 1
        if len(chosen) >= TARGET_NUM_MAX:
            break
    if len(chosen) < TARGET_NUM_MIN:  # 足りない時は単純に上から補完
        pool = [i for i, _, _ in ranked if i not in chosen]
        chosen += pool[:(TARGET_NUM_MIN - len(chosen))]
    return chosen

def select_scenes(profile, raw_data) -> List[int]:
    """メイン入口：ハイブリッド選考"""
    ranked = _rank_candidates(raw_data)
    if not ranked:
        raise ValueError("選考対象が見つかりません。raw_dataを確認してください。")

    if not USE_GPT_FINAL:
        return _pick_locally(ranked)

    # 上位KをGPTへ（Kが少なければ全部）
    ranked_topk = ranked[:TOPK_FOR_GPT]
    return _ask_gpt_to_pick(profile, raw_data, ranked_topk)
