# Event Mode (Painting Workshop) — techrie_demo

このモードは「ロボットと一緒に絵を描く」イベント運用向けの最小構成です。  
人の入力（Launchpad など）・ロボの自己提案・割り込みバランス・日記ログ（Diary）を簡潔に扱います。

## 起動

```
roslaunch techrie_demo event_mode.launch

    依存ノード（例）

        paint_executor.py（描画アクションの実体 /colors, /motion/play を駆動）

        motion_player.py（pose/seq/traj 再生）

        event_orchestrator.py（本モードの振る舞い統合）

        ask_and_confirm_node.py（0/1ボタンの確認サービス）

        reaction_router.py（発話表現のハブ：任意）

        diary_logger.py・photo_recorder.py（日記）

        human_input_bridge.py（ボタン→ /human/* へ）

        （任意）inactivity_watchdog.py → SULK_START を発火

主要トピック

人 → ロボ

    /human/invite … 「描いて！」（ボタンで指示）

    /human/show_art … 人が作品を見せる

    /human/praise … 人が褒める

    /human/pet … なでる

ロボ → 表現

    /robot_text (std_msgs/String) … 台詞

    /emotion/set (std_msgs/String) … 感情ラベル

    /motion/play (std_msgs/String) … pose:… / seq:… / traj:…

アクション

    paint_stroke (Action) … 描画実行（paint_executor.py）

    show_art (Action, 任意) … 見せる演出（無ければ pose:joy にフォールバック）

サービス

    ask_for_item(AskForItem) … 0/1 ボタンによる簡易確認（色OK / 置いたらOK 等）

いじけ（SULK）

    /interaction_events (techrie_demo/InteractionEvent)

        event_type=="SULK_START" を受けると、しょんぼり演出＆待機

        meta_json に {"dur": 秒数} を入れると長さ可変

条件分岐（振る舞いの流れ）

    人が /human/invite を送る
    → paint_stroke を開始 → 完了したら「できたよ！」 → （設定で）自動で見せる
    → 描画中に来た他の入力は「キュー」or「確率中断」ポリシーで処理

    人が /human/show_art を送る
    → ロボが興味→喜び＋軽いうなずきで反応

    人が /human/praise を送る
    → ロボが「ありがとう！」＋うなずき

    人が /human/pet を送る
    → ロボが「えへへ…」＋軽い姿勢リセット

    ロボの自己駆動（self-drive）

        selfdrive_enabled=true かつ
        直近の人入力から selfdrive_min_idle_sec 経過 & 前回提案から selfdrive_interval_sec 経過で
        「ちょっと描いてみてもいい？」と問いかけ

            ask_for_item("invite_confirm") を selfdrive_gate_wait_sec 待つ

                OK → 描画開始

                NG/無応答 → 「あとでにするね」

    SULK（いじけ）

        /interaction_events に SULK_START が来たら
        しょんぼり演出（首振り→reset→待機）→「しょんぼり」
        meta_json.dur で長さ調整可（既定 default_sulk_dur）

主なパラメータ（抜粋）
Param	既定値	説明
~announce_via_text	true	/robot_text と /emotion/set を出す
~paint_result_wait_sec	120.0	描画アクションの待ち時間
~auto_show_after_paint	true	描画後に自動で見せる
~auto_robot_praise_after_show	false	見せたあとロボからも褒める
~default_sulk_dur	12.0	SULK の標準時間（秒）
~selfdrive_enabled	true	自己駆動の有効/無効
~selfdrive_min_idle_sec	60.0	人入力がこれ以上なければ自己提案を検討
~selfdrive_interval_sec	90.0	前回提案から最低この間隔を空ける
~selfdrive_gate_wait_sec	12.0	自己提案時の 0/1 応答待ち
~interrupt_enabled	true	割り込みポリシーを有効化
~interrupt_policy	"queue"	"queue" or "cancel"
~interrupt_prob	0.4	"cancel" のとき中断に切替える確率
~diary_project	"making_the_moon_together"	日記メタ：プロジェクト
~diary_phase	"workshop_general"	日記メタ：フェーズ
~diary_goal	"co-create art pieces for the Moon"	日記メタ：目標

    くわしい実装は event_orchestrator.py を参照してください。

操作のヒント

    すぐ描かせたい

        /human/invite を送る（Launchpad の該当ボタン）

    描画中の人入力をもっと優先

        interrupt_policy: cancel と interrupt_prob を調整（例: 0.7 で優先的に人へ反応）

    自己提案の頻度調整

        selfdrive_min_idle_sec・selfdrive_interval_sec を増減

トラブルシュート

    ask_for_item が無い環境

        自己提案のゲートはデフォルトで NG扱い（一定時間待って流れる）

        実運用では ask_and_confirm_node.py を立てて OK/NG の 0/1 ボタンを反応させてください

    日記の写真が保存されない

        photo_recorder.py と /diary/snapshot のサービスを event モードでも起動してください
```

