#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
qa_batch_responder.py
- こどもたちの質問/メッセージに、Jedyが日本語でお返事をまとめて作成。
- techrie_demo/config.py があればそこから AzureOpenAI client と日付ディレクトリ解決を利用。
- なければ config/gpt_api.yaml を読みこんで AzureOpenAI client を生成（フォールバック）。

出力:
- object_images/YYYY/MM/DD/qa_answers.json に追記
- object_images/YYYY/MM/DD/qa_board.md に追記（掲示板用のMarkdown）
"""

import os
import sys
import re
import json
import glob
import argparse
import datetime as dt

# ===== パッケージルート解決 =====
try:
    from rospkg import RosPack
    PKG_ROOT = RosPack().get_path("techrie_demo")
except Exception as e:
    print("[qa_batch_responder] ERROR: ROSパッケージ techrie_demo の場所を解決できません。", file=sys.stderr)
    raise

if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

# ===== 依存 =====
import yaml  # pip install pyyaml
from openai import AzureOpenAI  # pip install --upgrade openai>=1.30

# ===== config.py の有無を確認 =====
CFG_AVAILABLE = False
try:
    # ~/catkin_ws/src/techrie_demo/config.py
    import config as tcfg  # パッケージ直下を import
    # 期待される属性: client(AzureOpenAI), resolve_paths_full(date_str=None)
    if hasattr(tcfg, "client") and hasattr(tcfg, "resolve_paths_full"):
        CFG_AVAILABLE = True
except Exception:
    CFG_AVAILABLE = False


# ------------------------------
# ヘルパ
# ------------------------------
def _ensure_dir(p):
    os.makedirs(p, exist_ok=True)

def _load_json_if(path, default=None):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    return default

def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def _find_latest_day_dir(base_dir):
    """object_images 以下の YYYY/MM/DD のうち、辞書順で最新を返す。無ければ None。"""
    if not os.path.isdir(base_dir):
        return None
    cand = []
    for y in sorted(os.listdir(base_dir)):
        if not (y.isdigit() and len(y) == 4): continue
        ydir = os.path.join(base_dir, y)
        if not os.path.isdir(ydir): continue
        for m in sorted(os.listdir(ydir)):
            if not (m.isdigit() and len(m) == 2): continue
            mdir = os.path.join(ydir, m)
            if not os.path.isdir(mdir): continue
            for d in sorted(os.listdir(mdir)):
                if not (d.isdigit() and len(d) == 2): continue
                ddir = os.path.join(mdir, d)
                if os.path.isdir(ddir):
                    cand.append((y, m, d, ddir))
    if not cand:
        return None
    cand.sort(key=lambda x: (x[0], x[1], x[2]))
    return cand[-1][-1]

def _resolve_day_paths_with_config(date_str=None):
    """
    config.py の resolve_paths_full を使って標準パスを得た上で、
    captioned.json の命名差（*_captioned.json or captioned.json）にも対応。
    """
    paths = tcfg.resolve_paths_full(date_str=date_str)
    day_dir = paths["day_dir"]
    _ensure_dir(day_dir)

    # captioned: {date}_captioned.json or captioned.json の両方に対応
    date_iso = dt.date.today().isoformat() if date_str is None else dt.date.fromisoformat(date_str).isoformat()
    preferred = os.path.join(day_dir, f"{date_iso}_captioned.json")
    alt = os.path.join(day_dir, "captioned.json")

    captioned_json = preferred if os.path.exists(preferred) else (alt if os.path.exists(alt) else preferred)

    out = {
        "day_dir": day_dir,
        "profile_json": os.path.join(PKG_ROOT, "config", "Jedy.json"),
        "diary_json": os.path.join(day_dir, "diary.json"),
        "captioned_json": captioned_json,
        "qa_json": os.path.join(day_dir, "qa_answers.json"),
        "qa_board_md": os.path.join(day_dir, "qa_board.md"),
    }
    return out

def _resolve_day_paths_plain(date_str=None):
    """
    config.py が無い場合のフォールバック: object_images/YYYY/MM/DD を自前で解決。
    """
    base = os.path.join(PKG_ROOT, "object_images")
    if date_str:
        d = dt.date.fromisoformat(date_str)
        day_dir = os.path.join(base, f"{d.year:04d}", f"{d.month:02d}", f"{d.day:02d}")
    else:
        day_dir = _find_latest_day_dir(base)
        if day_dir is None:
            today = dt.date.today()
            day_dir = os.path.join(base, f"{today.year:04d}", f"{today.month:02d}", f"{today.day:02d}")
    _ensure_dir(day_dir)

    # captioned は *_captioned.json or captioned.json を優先探索
    cands = sorted(glob.glob(os.path.join(day_dir, "*captioned.json")))
    captioned_json = cands[-1] if cands else os.path.join(day_dir, "captioned.json")

    return {
        "day_dir": day_dir,
        "profile_json": os.path.join(PKG_ROOT, "config", "Jedy.json"),
        "diary_json": os.path.join(day_dir, "diary.json"),
        "captioned_json": captioned_json,
        "qa_json": os.path.join(day_dir, "qa_answers.json"),
        "qa_board_md": os.path.join(day_dir, "qa_board.md"),
    }

def resolve_day_paths(date_str=None):
    if CFG_AVAILABLE:
        return _resolve_day_paths_with_config(date_str)
    return _resolve_day_paths_plain(date_str)

def _build_context_text(profile, diary, captions):
    """
    GPT に渡す『状況説明（system プロンプト）』を日本語で構築。
    """
    lines = []
    name = (profile or {}).get("name", "Jedy")
    persona = (profile or {}).get("personality", [])
    hobbies = (profile or {}).get("hobbies", [])

    lines.append(f"あなたは5歳のロボット『{name}』として、中高生に日本語で答えます。")
    if persona:
        lines.append("性格: " + "、".join(persona))
    if hobbies:
        lines.append("趣味: " + "、".join(hobbies))

    # diary.json の要約・本文
    if diary and isinstance(diary, dict):
        summ = diary.get("summary") or diary.get("diary") or ""
        if summ:
            summ = re.sub(r"\s+", " ", summ)
            lines.append("最近の出来事要約: " + (summ[:400] + ("…" if len(summ) > 400 else "")))

    # captioned（最近のスナップ最大5件）
    if captions:
        # captions は list or dict どちらもありうるので吸収
        items = []
        if isinstance(captions, list):
            items = captions
        elif isinstance(captions, dict):
            items = captions.get("items") or captions.get("data") or []
        if items:
            lines.append("最近のスナップ（最大5件）:")
            for e in items[-5:]:
                cap = e.get("caption") or e.get("text") or ""
                ts  = e.get("timestamp") or e.get("time") or ""
                if cap:
                    cap_line = f"- {ts}: {cap}"
                    lines.append(cap_line[:200])

    # 口調のガイド
    lines.append("注意: 中高生向けに、やさしく、具体的に、100字以内で簡潔に。")
    lines.append("一人称は『ぼく』。顔文字や過度な擬態語は控えめに。")

    return "\n".join(lines)

def _load_azure_client_from_yaml():
    yml = _load_yaml(os.path.join(PKG_ROOT, "config", "gpt_api.yaml"))
    chat = yml.get("chat", {})
    api_key = chat.get("api_key")
    api_version = chat.get("api_version", "2024-12-01-preview")
    azure_endpoint = chat.get("azure_endpoint")
    deployment = chat.get("deployment", "o1")
    temperature = float(chat.get("temperature", 0.7)) if chat.get("allow_temperature", True) else None
    timeout_sec = float(chat.get("timeout_sec", 60.0))

    if not (api_key and azure_endpoint and deployment):
        raise RuntimeError("config/gpt_api.yaml の chat.api_key / azure_endpoint / deployment を確認してください。")

    client = AzureOpenAI(api_key=api_key, api_version=api_version, azure_endpoint=azure_endpoint)
    return client, deployment, temperature, timeout_sec

def _load_client_and_model():
    """
    1) config.py の client を使う（model は 'o1' を既定）
    2) なければ gpt_api.yaml から生成
    """
    if CFG_AVAILABLE:
        client = tcfg.client
        model = "o1"  # config.py が base_url でデプロイ固定にしていても model は合わせておく
        temperature = 0.7
        timeout_sec = 60.0
        return client, model, temperature, timeout_sec
    return _load_azure_client_from_yaml()

def _ask_gpt(client, model, system_text, user_text, temperature=None, timeout_sec=8.0):
    timeout_sec = 60
    messages = [
        {"role": "system", "content": system_text},
        {"role": "user",   "content": user_text},
    ]
    kwargs = dict(model=model, messages=messages)
    if temperature is not None:
        kwargs["temperature"] = float(temperature)
    # openai>=1.30 は timeout 引数サポート
    resp = client.chat.completions.create(**kwargs, timeout=timeout_sec)
    return (resp.choices[0].message.content or "").strip()

def _read_questions_from_file(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines()]
    return [ln for ln in lines if ln]

def _read_questions_stdin():
    print("質問/メッセージを1行ずつ入力してください（EOF: Ctrl-D）:", file=sys.stderr)
    buf = sys.stdin.read()
    lines = [ln.strip() for ln in buf.splitlines()]
    return [ln for ln in lines if ln]


# ------------------------------
# 本体
# ------------------------------
def main():
    ap = argparse.ArgumentParser(description="Jedyの一括お返事メーカー（質問→日本語回答）")
    ap.add_argument("--in", dest="infile", default=None, help="1行1問のテキストファイル。省略時は標準入力")
    ap.add_argument("--date", dest="date", default=None, help="対象日 (YYYY-MM-DD)。省略時は最新 or 今日を作成")
    ap.add_argument("--title", dest="title", default="Jedyへのお返事", help="掲示板タイトル")
    ap.add_argument("--max", dest="maxn", type=int, default=100, help="最大質問数")
    ap.add_argument("--dry-run", action="store_true", help="保存せず標準出力のみ")
    args = ap.parse_args()

    # 対象日ディレクトリ＆入出力パス
    paths = resolve_day_paths(args.date)
    day_dir = paths["day_dir"]
    _ensure_dir(day_dir)

    # 文脈
    profile = _load_json_if(paths["profile_json"], {})
    diary   = _load_json_if(paths["diary_json"], {})
    captions= _load_json_if(paths["captioned_json"], [])

    # 質問読み込み
    if args.infile:
        questions = _read_questions_from_file(args.infile)
    else:
        questions = _read_questions_stdin()
    if not questions:
        print("[qa_batch_responder] 質問がありません。--in でファイルを渡すか、標準入力で入力してください。", file=sys.stderr)
        return
    if len(questions) > args.maxn:
        questions = questions[:args.maxn]

    # GPT クライアント
    client, model, temperature, timeout_sec = _load_client_and_model()

    # system プロンプト
    system_text = _build_context_text(profile, diary, captions)

    # 既存の JSON を読み込み（追記運用）
    qa_json_path = paths["qa_json"]
    qa_board_md  = paths["qa_board_md"]
    existed = _load_json_if(qa_json_path, {"items": []})
    items = existed.get("items", [])

    # 生成
    new_items = []
    for q in questions:
        user_text = (
            "以下は中高生からの質問/メッセージです。\n"
            "やさしく・具体的に・100字以内で答えてください。\n\n"
            f"Q: {q}\n"
            "A:"
        )
        try:
            #ans = _ask_gpt(client, model, system_text, user_text,temperature=temperature, timeout_sec=timeout_sec)
            ans = _ask_gpt(client, model, system_text, user_text)
        except Exception as e:
            ans = f"(生成に失敗しました: {e})"

        one = {
            "timestamp": dt.datetime.now().isoformat(timespec="seconds"),
            "question": q,
            "answer": ans
        }
        items.append(one)
        new_items.append(one)

    if args.dry_run:
        print("# " + args.title)
        for it in new_items:
            print(f"- Q: {it['question']}")
            print(f"  A: {it['answer']}\n")
        return

    # JSON 保存
    out = {"title": args.title, "date_dir": day_dir, "items": items}
    with open(qa_json_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    # Markdown 掲示板 追記
    header_needed = not os.path.exists(qa_board_md) or os.stat(qa_board_md).st_size == 0
    with open(qa_board_md, "a", encoding="utf-8") as f:
        if header_needed:
            f.write(f"# {args.title}\n")
            f.write(f"- 保存先: {day_dir}\n")
            f.write(f"- 生成時刻: {dt.datetime.now().isoformat(timespec='seconds')}\n\n")
        for it in new_items:
            f.write(f"**Q:** {it['question']}\n\n")
            f.write(f"**A (Jedy):** {it['answer']}\n\n---\n")

    print(f"[qa_batch_responder] 保存: {qa_json_path}")
    print(f"[qa_batch_responder] 掲示板: {qa_board_md}")


if __name__ == "__main__":
    main()
