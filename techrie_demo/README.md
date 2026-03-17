# techrie_demo

`techrie_demo` は、ロボットと人の共創・対話デモ（挨拶 / イベント / 日次運用 / 終了挨拶）を行うための ROS パッケージです。

- プロジェクト実施概要(JSK内部のみ閲覧可）：

https://drive.google.com/drive/folders/1XYhy79nsa9jBod3T8mexUpDNlz_K7bew?usp=sharing

- イベント実施時のスライド（JSK内部のみ閲覧可能)

https://docs.google.com/presentation/d/179zz4kf8Qx043eTgupcHrlLExAfg9EgkcPOddYiXf4c/edit?usp=sharing

- 説明スライド（JSK内部のみ閲覧可能）

https://docs.google.com/presentation/d/1qYzKMFAmBUZeDUsA0WqrwGvUIunvp0UAaEu4uHVW_dw/edit?usp=sharing

---

## 1. 起動するlaunch

このパッケージでは、主に次の 4 つの launch を使います。

- `greeting_mode.launch`（開始時の挨拶・初回イベント用）
- `event_mode.launch`（イベント本体・共創/お絵描き運用）
- `daily_mode.launch`（日次運用・記録含む）
- `end_greeting_mode.launch`（終了時の挨拶・最終回イベント用）

### 使い分け
- **最初の挨拶を含む** → `greeting_mode.launch`
- **イベント本体（人入力・描画・反応）を動かしたい** → `event_mode.launch`
- **通常時の運用を含む** → `daily_mode.launch`
- **終了の挨拶を含む** → `end_greeting_mode.launch`

---

## 2. セットアップ（初回）

### 前提
- ROS 1（catkin workspace）
- `techrie_demo` が `src/` 配下にあること
- 必要なハードウェア/外部ノード（カメラ、入力デバイス、ロボット制御系）は別途起動

### 依存解決（推奨）
ワークスペースのルートで:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### ビルド
```bash
cd ~/tmp_ws
catkin build
source devel/setup.bash
```

---

## 3. 起動方法（基本）
まず、ロボット(Jedy)の中で
```
roslaunch jedy_play new_jedy.launch use_camera:=true
```
をしてから以下のいずれかを実行する

```bash
# 初回イベントの挨拶
roslaunch techrie_demo greeting_mode.launch

# イベントのとき
roslaunch techrie_demo event_mode.launch

# 日常導入のとき
roslaunch techrie_demo daily_mode.launch

# 最終回の挨拶
roslaunch techrie_demo end_greeting_mode.launch
```

---

## 4. Event Mode（Painting Workshop）の概要

`event_mode.launch` は、「ロボットと一緒に絵を描く」イベント運用向けの最小構成です。  
人の入力（Launchpad など）・ロボの自己提案・割り込みバランス・日記ログ（Diary）を扱います。

### 4.1 主な依存ノード（例）
- `paint_executor.py`（描画アクションの実体 /colors, /motion/play を駆動）
- `motion_player.py`（pose/seq/traj 再生）
- `event_orchestrator.py`（本モードの振る舞い統合）
- `ask_and_confirm_node.py`（0/1 ボタンの確認サービス）
- `reaction_router.py`（発話表現のハブ：任意）
- `diary_logger.py`・`photo_recorder.py`（日記）
- `human_input_bridge.py`（ボタン→ `/human/*` へ）
- （任意）`inactivity_watchdog.py` → `SULK_START` を発火

---

## 5. 主要トピック（まず見るところ）

### 5.1 人 → ロボット（入力系）
`event_mode` では、以下の入力トピックをきっかけにロボットの行動を進行します。

- `/human/invite` … 「描いて！」（ボタンで指示）
- `/human/show_art` … 人が作品を見せる
- `/human/praise` … 人が褒める
- `/human/pet` … なでる

> 実際の入力デバイス（例: ボタン、Launchpad）からの変換は `human_input_bridge.py` などが担当します。

### 5.2 ロボット → 表現（出力系）
- `/robot_text` (`std_msgs/String`) … 台詞
- `/emotion/set` (`std_msgs/String`) … 感情ラベル
- `/motion/play` (`std_msgs/String`) … `pose:...` / `seq:...` / `traj:...`

### 5.3 イベント通知
- `/interaction_events` (`techrie_demo/InteractionEvent`)
  - `event_type == "SULK_START"` を受けると、しょんぼり演出＆待機
  - `meta_json` に `{"dur": 秒数}` を入れると長さ可変

---

## 6. アクション / サービス（event_mode）

### 6.1 アクション
- `paint_stroke` (Action) … 描画実行（`paint_executor.py`）
- `show_art` (Action, 任意) … 見せる演出（無ければ `pose:joy` にフォールバック）

### 6.2 サービス
- `ask_for_item(AskForItem)` … 0/1 ボタンによる簡易確認（色OK / 置いたらOK 等）
- `/diary/snapshot` (`std_srvs/Trigger`) … 現在画像をスナップショット保存（`photo_recorder.py`）

---

## 7. event_mode の振る舞い

### 人が `/human/invite` を送る
→ `paint_stroke` を開始  
→ 完了したら「できたよ！」  
→ （設定で）自動で見せる  
→ 描画中に来た他の入力は「キュー」or「確率中断」ポリシーで処理

### 人が `/human/show_art` を送る
→ ロボが興味→喜び＋軽いうなずきで反応

### 人が `/human/praise` を送る
→ ロボが「ありがとう！」＋うなずき

### 人が `/human/pet` を送る
→ ロボが「なでなで…」＋軽い姿勢リセット

### ロボの自己駆動（self-drive）
`selfdrive_enabled=true` かつ

- 直近の人入力から `selfdrive_min_idle_sec` 経過
- 前回提案から `selfdrive_interval_sec` 経過

のとき、「描きたい！」と誘う

`ask_for_item("invite_confirm")` を `selfdrive_gate_wait_sec` 秒待って

- OK → 描画開始
- NG / 無応答 → 「あとでにするね」

### SULK（いじけ）
`/interaction_events` に `SULK_START` が来たら

- しょんぼり演出（首振り→reset→待機）→「しょんぼり」
- `meta_json.dur` で長さ調整可（既定 `default_sulk_dur`）

---

## 8. 主なパラメータ（event_mode 抜粋）

| Param | 既定値 | 説明 |
|---|---:|---|
| `~announce_via_text` | `true` | `/robot_text` と `/emotion/set` を出す |
| `~paint_result_wait_sec` | `120.0` | 描画アクションの待ち時間 |
| `~auto_show_after_paint` | `true` | 描画後に自動で見せる |
| `~auto_robot_praise_after_show` | `false` | 見せたあとロボからも褒める |
| `~default_sulk_dur` | `12.0` | SULK の標準時間（秒） |
| `~selfdrive_enabled` | `true` | 自己駆動の有効/無効 |
| `~selfdrive_min_idle_sec` | `60.0` | 人入力がこれ以上なければ自己提案を検討 |
| `~selfdrive_interval_sec` | `90.0` | 前回提案から最低この間隔を空ける |
| `~selfdrive_gate_wait_sec` | `12.0` | 自己提案時の 0/1 応答待ち |
| `~interrupt_enabled` | `true` | 割り込みポリシーを有効化 |
| `~interrupt_policy` | `"queue"` | `"queue"` or `"cancel"` |
| `~interrupt_prob` | `0.4` | `"cancel"` のとき中断に切替える確率 |
| `~diary_project` | `"making_the_moon_together"` | 日記メタ：プロジェクト |
| `~diary_phase` | `"workshop_general"` | 日記メタ：フェーズ |
| `~diary_goal` | `"co-create art pieces for the Moon"` | 日記メタ：目標 |

> 詳しい実装は `event_orchestrator.py` を参照してください。

---

## 9. Developer Tools: Pose authoring（姿勢作成・調整）

`techrie_demo` では、腕ポーズや簡単な軌道を `config/arm_poses.yaml` / `motions/*.json` として管理します。  
以下のスクリプトは **本番 launch とは別の開発補助ツール** です。

### 9.1 単発ポーズを保存する（`joint_save.py`）
- Script: `scripts/joint_save.py`
- Input topic: `/joint_states`
- Output: `config/arm_poses.yaml`（`poses` セクション）

```bash
rosrun techrie_demo joint_save.py
```

- 実行後、Enter で現在姿勢を取り込み
- ポーズ名を入力して保存

### 9.2 軌道を記録する（`joint_record.py`）
- Script: `scripts/joint_record.py`
- Input topic: `/joint_states`
- Output: `motions/<name>.json`

```bash
rosrun techrie_demo joint_record.py
```

- Enter で記録開始
- Enter で停止 → 保存

### 9.3 記録した軌道を YAML に変換する（`json_to_yaml_poses.py`）
- Script: `scripts/json_to_yaml_poses.py`
- Input: `motions/<name>.json`
- Output: `config/arm_poses.yaml`（`poses`, `sequences`）

```bash
rosrun techrie_demo json_to_yaml_poses.py <name> [step_dur]
```

### 9.4 姿勢/位置の確認（`watch_pose.py`）
- Script: `scripts/watch_pose.py`
- 用途: `/paint_position` などの位置変化を確認（デバッグ兼調整）

---

## 10. Developer Tools: Test / Debug（テスト・デバッグ）

### 10.1 入力デバイス確認（Launchpad / Joy）
- `scripts/joy_probe.py`  
  `/joy` の押下ボタン番号を確認（ROS経由）

```bash
rosrun techrie_demo joy_probe.py
```

- `scripts/joy_test.py`, `scripts/joy_test2.py`  
  Launchpad 単体のLED確認（ROS非依存、ローカル環境向け）

### 10.2 アクション / パイプライン疎通確認
- `scripts/ping_actions.py`  
  `invite` / `paint_stroke` アクションの単発テスト

```bash
rosrun techrie_demo ping_actions.py invite
rosrun techrie_demo ping_actions.py paint
```

- `scripts/smoke_paint.py`  
  paint系のスモークテスト（イベント系の疎通確認）

- `scripts/check_techrie.py`  
  トピック・サービス・アクションの生存確認（環境チェック）

### 10.3 まず見るべき確認コマンド
```bash
# ノード一覧
rosnode list

# human / robot / interaction だけ見る
rostopic list | egrep "human|robot|interaction"

# event 通知
rostopic echo /interaction_events

# ロボットの台詞
rostopic echo /robot_text

# 接続を可視化
rqt_graph
```

---

## 11. Diary（絵日記）pipeline

`techrie_demo` には、イベントログからロボットの日記を生成する機能があります。  
**実行時のログ収集（ROS）** と **生成パイプライン（Python）** の 2 段構成です。

### 11.1 Runtime nodes（ROS）

#### `photo_recorder.py`
- Subscribes: `~image_topic`（default: `/camera/color/image_raw`）
- Service: `/diary/snapshot`
- Saves snapshots to: `~/.ros/techrie_demo/object_images/YYYY/MM/DD/*.jpg`
- `~save_root` パラメータで保存先を上書き可能（default: `~/.ros/techrie_demo/object_images`）

#### `diary_logger.py`
- Subscribes: `/interaction_events`（必要に応じて `/robot_text` など）
- Calls: `/diary/snapshot`（イベント時にスナップショットを取る）
- Appends diary log to: `~/.ros/techrie_demo/diary/...`
- DB: `~/.ros/techrie_demo/diary.db`

#### `diary_pipeline_service.py`
- Service: `/diary/make_today`
- Runs: `main_pipeline.py`（scene selection → caption → diary text → image generation）
- Outputs（例）:
  - `diary.json`
  - `diary_image.png`
  - `diary_combined.png`

#### `diary_switch.py`
- Subscribes: `/joy`
- Trigger button → `/diary/make_today` を呼ぶための補助ノード

### 11.2 Offline pipeline scripts（任意）
- `main_pipeline.py`
- `step1_select_scene.py`
- `step2_generate_caption.py`
- `step3_generate_diary.py`
- `step4_generate_image.py`
- `step5_merge_image_and_text.py`
- `gpt_profile_reflect.py`

### 11.3 API 設定（Azure OpenAI）
Diary 生成（caption / text / image）には API 設定が必要です。

- ローカルファイル（gitignore）: `config/gpt_api.yaml`
- または環境変数（例: `TECHRIE_TEXT_*`, `TECHRIE_IMAGE_*`）

テンプレート:
```bash
cp config/gpt_api.yaml.example config/gpt_api.yaml
```

> **注意**: `config/gpt_api.yaml` は Git 管理していません。APIキーを公開しないように気をつけてください。

`config.py` は以下を参照します。

- text: `api_key`, `api_version`, `base_url`
- image: `api_key`, `api_version`, `azure_endpoint`

環境変数を使う場合:
- `TECHRIE_TEXT_API_KEY`
- `TECHRIE_TEXT_API_VERSION`
- `TECHRIE_TEXT_BASE_URL`
- `TECHRIE_IMAGE_API_KEY`
- `TECHRIE_IMAGE_API_VERSION`
- `TECHRIE_IMAGE_AZURE_ENDPOINT`

---

## 11.4 実機なしデモ: miniature event + diary demo

`techrie_demo` には、**実機なし**で「小さな相互作用の流れ → ロボット日記生成」まで試せるデモがあります。  
このデモでは、ROS の event mode をそのまま再現する代わりに、

1. 小さな相互作用シナリオ（JSON）
2. 各場面に対応する画像
3. Diary pipeline

を使って、`techrie_demo` の特徴である

- 人の働きかけへの反応
- ロボット側の自己提案
- 小さな出来事の列
- その出来事が日記として言語化されること

を、実機なしで確認できます。

### 関連ファイル
- `run_story_demo.py`  
  小さな相互作用シナリオを replay し、その結果を `scenes_json` に保存したうえで Diary pipeline を呼ぶ
- `action_policy.py`  
  実機・ROS 非依存の簡易行動提案ロジック
- `sample_story_demo.json`  
  サンプルシナリオ
- `run_diary_demo.py`  
  Diary pipeline 単体の実行用
- `demo_assets/story_demo/scene1.jpg` 〜 `scene4.jpg`  
  サンプル画像

### 事前準備
#### API 設定
Diary 生成には Azure OpenAI の設定が必要です。  
`config/gpt_api.yaml` を用意してください。

例:
```yaml
provider: azure

text:
  api_key: "YOUR_TEXT_KEY"
  api_version: "2024-12-01-preview"
  base_url: "https://<your-text-resource>.openai.azure.com/openai/deployments/<your-text-deployment>"

image:
  api_key: "YOUR_IMAGE_KEY"
  api_version: "2025-04-01-preview"
  azure_endpoint: "https://<your-image-resource>.openai.azure.com/"
```

> `config/gpt_api.yaml` は `.gitignore` 対象です。公開しないでください。

#### サンプル画像の配置
サンプル画像そのものは repo 内の `demo_assets/story_demo/` に置いてあります。  
ただし実行時には、Diary pipeline が **`~/.ros/techrie_demo/object_images/YYYY/MM/DD/`** を参照するため、その日の `day_dir` にコピーする必要があります。

例: `2026-03-16` の場合

```bash
mkdir -p ~/.ros/techrie_demo/object_images/2026/03/16
cp demo_assets/story_demo/scene1.jpg ~/.ros/techrie_demo/object_images/2026/03/16/
cp demo_assets/story_demo/scene2.jpg ~/.ros/techrie_demo/object_images/2026/03/16/
cp demo_assets/story_demo/scene3.jpg ~/.ros/techrie_demo/object_images/2026/03/16/
cp demo_assets/story_demo/scene4.jpg ~/.ros/techrie_demo/object_images/2026/03/16/
```

### まず相互作用リプレイだけ書き出す
```bash
python run_story_demo.py   --date 2026-03-16   --story sample_story_demo.json   --write-scenes-only
```

これにより、以下が生成されます。

- `~/.ros/techrie_demo/object_images/2026/03/16/2026-03-16.json`

この JSON は、miniature interaction replay の結果を Diary pipeline 用 `scenes_json` として保存したものです。

### 日記本文まで生成する
画像生成をまだ行わず、本文まで確認したい場合:

```bash
python run_story_demo.py   --date 2026-03-16   --story sample_story_demo.json   --skip-image-gen   --skip-merge   --skip-profile-reflect
```

生成物:
- `2026-03-16_captioned.json`
- `diary.json`

### 日記画像まで生成する
画像生成 API 設定も正しければ、`--skip-image-gen` を外して実行できます。

```bash
python run_story_demo.py   --date 2026-03-16   --story sample_story_demo.json   --skip-merge   --skip-profile-reflect
```

生成物:
- `diary_image.png`

### 合成画像まで生成する
```bash
python run_story_demo.py   --date 2026-03-16   --story sample_story_demo.json   --skip-profile-reflect
```

生成物:
- `diary_combined.png`

### Diary pipeline 単体で試す
相互作用 replay を使わず、手で `scenes_json` を作って Diary pipeline だけ試す場合は:

```bash
python run_diary_demo.py --date 2026-03-16 --init-sample --dry-run
python run_diary_demo.py --date 2026-03-16 --skip-image-gen --skip-merge --skip-profile-reflect
```

### 補足
- 実行時の入力・出力は source tree ではなく `~/.ros/techrie_demo/object_images/...` に保存されます。
- repo 内の `demo_assets/story_demo/` は、**再現用のサンプル画像置き場**です。
- 生成物そのものは git 管理せず、`~/.ros` 側に出す想定です。

---

## 12. 保存先（生成物）

このパッケージは、生成物を **source tree ではなく `~/.ros` 配下** に保存する設計です。

主な保存先:
- `~/.ros/techrie_demo/object_images/` … 画像・スナップショット
- `~/.ros/techrie_demo/diary/` … 日記（Markdown など）
- `~/.ros/techrie_demo/diary.db` … 日記DB
- `~/.ros/techrie_demo/profile/` … 実行時プロフィール状態

---

## 13. 開発のために

初めてコードを読む場合は、次の順番がおすすめです。

1. `launch/event_mode.launch`  
   どのノードが起動されるかを把握する

2. `nodes/event/event_orchestrator.py`  
   イベント運用の中心ロジックを読む

3. `nodes/hw/motion_player.py`  
   モーション実行の流れを把握する

4. `nodes/social/reaction_router.py`  
   発話/感情/反応の出し方を把握する

5. `nodes/logging/photo_recorder.py`, `nodes/logging/diary_logger.py`  
   記録系の保存先とフォーマットを確認する

---

## 14. トラブルシュート（最小）

### `config/gpt_api.yaml` がない
テンプレートから作成してください:

```bash
cp config/gpt_api.yaml.example config/gpt_api.yaml
```

### `roslaunch` でノードが見つからない
- `catkin_make` 済みか
- `source devel/setup.bash` 済みか
- `techrie_demo` が catkin workspace の `src/` 配下にあるか

### 入力しても反応しない
- `human_input_bridge.py` が起動しているか
- `/human/*` トピックに入力が流れているか（`rostopic echo` で確認）
- `event_orchestrator.py` が起動しているか

### `ask_for_item` が無い環境
- 自己提案のゲートはデフォルトで NG 扱い（一定時間待って流れる）
- 実運用では `ask_and_confirm_node.py` を立てて、OK/NG の 0/1 ボタンを反応させてください

### 日記の写真が保存されない
- `photo_recorder.py` が起動しているか
- `/diary/snapshot` サービスが見えているか（`rosservice list | grep diary`）
- `~image_topic` に画像が来ているか（`rostopic echo -n1 <image_topic>`）

---

## 15. 補足

- 実行時に使う外部ノード・ハードウェア（カメラ、入力装置、ロボット実機側ノード）は環境依存です
- 詳細なパラメータは各 `launch/*.launch` と各ノード内の `rospy.get_param(...)` を参照してください
