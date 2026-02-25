# step3_generate_diary.py
import json
from config import client


def generate_diary_text(profile, raw_data, paths):
    summaries = []
    for entry in raw_data:
        if "caption" not in entry:
            continue

        cap = entry["caption"]
        mode = entry.get("mode", "不明")
        emo = entry.get("emotion", "不明")

        rs = entry.get("robot_state", {})
        rs_text = (
            f"興味: {rs.get('interest', '?'):.2f}, 疲労: {rs.get('fatigue', '?'):.2f}, "
            f"最後の行動: {rs.get('last_action', '不明')}, モード: {rs.get('mode', '不明')}, "
            f"社会性: {rs.get('sociality_score', '?'):.2f}"
        )

        hs = entry.get("human_state", {})
        hs_text = (
            f"感情: {hs.get('emotion', '不明')}, 行動: {hs.get('action', '不明')}, "
            f"ニーズ: {hs.get('needs', '不明')}, 交流: {hs.get('interaction', '不明')}, "
            f"視線: {hs.get('gaze_target', '不明')}"
        )

        extra = entry.get("extra_info", {})
        wx = f"天気: {extra.get('weather', '不明')}, 気温: {extra.get('temperature', '不明')}"

        summary = f"- {cap} (mode: {mode}, emotion: {emo})\n  ロボットの状態: {rs_text}\n  人の状態: {hs_text}\n  {wx}"
        summaries.append(summary)

    if not summaries:
        raise ValueError("caption付きのエントリが見つかりません。")

    summary_text = "\n".join(summaries)

    prompt_diary = f"""
    ぼくは{profile['age']}さいのロボット、{profile['name']}。
    {"、".join(profile['personality'])}なところがあるよ。

    きょうはこんなことがあったんだ：

    {summary_text}

    この1日をふりかえって、子どもに読んでもらうための日記を書いてください。
    文体の例：「きょうは○○したよ。とってもたのしかったな。△△もやったんだ。またあしたね！ – {profile['name']}」
    この文体にならって、一人称「ぼく」で、やさしい語調で7-10文程度で書いてください。
    """

    diary_response = client.chat.completions.create(
        model="o1",
        messages=[{"role": "user", "content": prompt_diary}],
    )

    diary_text = diary_response.choices[0].message.content.strip()

    with open(paths["diary_output_path"], "w", encoding="utf-8") as f:
        json.dump({"date": paths["date_str"], "text": diary_text}, f, ensure_ascii=False, indent=2)

    print(f"✅ 絵日記を {paths['diary_output_path']} に保存しました。")
    return diary_text
