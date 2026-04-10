# gpt_profile_reflect.py
import os
import json
from pathlib import Path
from datetime import datetime, timedelta
from config import get_chat_client, data_root, pkg_root

def run_gpt_profile_reflect(start_date: str=None):
    # === 設定 ===
    profile_path = Path("../profile/Jedy.json")
    diary_root = Path("../object_images")
    LOOKBACK_DAYS = 7

    client = get_chat_client()
    profile_path = Path(pkg_root()) / "profile" / "Jedy.json"
    diary_root = Path(data_root()) / "object_images"

    # === 日付決定 ===
    if start_date:
        base_date = datetime.strptime(start_date, "%Y-%m-%d").date()
    else:
        base_date = datetime.now().date()

    dates = [(base_date - timedelta(days=i)).strftime("%Y-%m-%d") for i in range(LOOKBACK_DAYS)]
    all_texts = []
    for date in dates:
        y, m, d = date.split("-")
        diary_path = diary_root / y / m / d / "diary.json"
        if diary_path.exists():
            with open(diary_path, "r") as f:
                data = json.load(f)
                all_texts.append(f"[{date}] {data.get('text', '')}")

    if not all_texts:
        print("⚠️ 日記が見つかりません。処理を終了します。")
        return

    joined_texts = "\n".join(all_texts)

    # === GPTにプロフィール更新案を依頼 ===
    # === GPTに分析を依頼 ===
    prompt = f"""
    以下はロボットJedyが今週書いた日記の一覧です。
    この1週間でJedyの関心・好きなもの・得意なこと・性格にどんな変化があったかを分析し、プロフィールを以下の4項目に分けて更新してください：

    - favorite_things: [追加・変更がある場合のみ]
    - skills: [新しく得た能力や経験]
    - hobbies: [繰り返し登場する活動]
    - personality: [性格として印象的な変化や傾向があれば]

    出力は JSON 形式でお願いします。

    {chr(10).join(all_texts)}
    """

    response = client.chat.completions.create(
        model="o1",
        messages=[
            {"role": "user", "content": prompt}
        ]
    )

    try:
        suggestion = json.loads(response.choices[0].message.content)
    except json.JSONDecodeError:
        print("⚠️ GPTの出力がJSON形式ではありません。処理中止。")
        return

    # === GPTに recent_memories の要約を依頼 ===
    memory_prompt = f"""
    以下はロボットJedyが今週書いた日記の内容です。
    子どもにも伝えられるような、3〜5個の思い出を短く箇条書きでまとめてください。
    一人称「ぼく」で、やさしく、具体的に書いてください。
    出力形式:
    - ○○したこと
    - △△だったこと

    {joined_texts}
    """

    memory_response = client.chat.completions.create(
        model="o1",
        messages=[
            {"role": "user", "content": memory_prompt}
        ]
    )

    memory_lines = memory_response.choices[0].message.content.strip().split("\n")
    recent_memories = [line.strip("- ") for line in memory_lines if line.strip().startswith("- ") or line.strip().startswith("・")]


    # === プロフィールの読み込みと更新 ===
    with open(profile_path, "r") as f:
        profile = json.load(f)

    for field in ["favorite_things", "skills", "hobbies", "personality"]:
        new_items = suggestion.get(field, [])
        if new_items:
            profile.setdefault(field, [])
            before = set(profile[field])
            after = before.union(new_items)
            profile[field] = list(after)
            print(f"✅ {field} に追加: {list(after - before)}")

    with open(profile_path, "w") as f:
        json.dump(profile, f, ensure_ascii=False, indent=2)

    print("🧠 プロフィールの更新が完了しました。")
