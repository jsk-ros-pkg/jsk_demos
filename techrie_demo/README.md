# techrie_demo

`techrie_demo` は、ロボットと人の共創・対話デモ（挨拶 / イベント / 日次運用 / 終了挨拶）を行うための ROS パッケージです。  
本パッケージでは、主に以下の4つの運用 launch を入口として使います。

- `greeting_mode.launch`（開始時の挨拶）
- `event_mode.launch`（イベント本体・共創/お絵描き運用）
- `daily_mode.launch`（日次運用・記録含む）
- `end_greeting_mode.launch`（終了時の挨拶）

この README は、**初めて触る人が「どう起動して」「どのノード/トピックを見ればよいか」** をすぐ把握できるようにまとめています。

---

## 1. パッケージの位置づけ

`techrie_demo` は、次のような役割を組み合わせた構成になっています。

- **行動実行**（姿勢・モーション再生）
- **イベント進行**（人入力 / 自己提案 / 割り込み処理）
- **表現**（発話・感情ラベル）
- **記録**（写真・日記ログ）
- **橋渡し**（ボタン入力やイベント通知）

---

## 2. 運用モード（launch）

### `greeting_mode.launch`
開始時の挨拶デモ用。  
運用の立ち上がりで、ロボットの基本挨拶や短い反応を動かすための launch です。

### `event_mode.launch`
イベント本体（共創・お絵描き運用）の中心となる launch。  
人の入力（例: ボタン）に応じて、ロボットが描画・反応・自己提案を行います。  
日記記録系（写真 / diary）もこのモードで使う想定です。

### `daily_mode.launch`
日次運用用。  
日々の記録・状態遷移を含む運用をまとめて起動するための launch です。

### `end_greeting_mode.launch`
終了時の挨拶デモ用。  
イベント終了時の締めの挨拶や短い演出に使います。

---

## 3. セットアップ（初回）

### 前提
- ROS 1（catkin workspace）
- `techrie_demo` が `src/` 以下にあること
- 必要なハードウェア/外部ノード（カメラ、入力デバイス、ロボット制御系）は別途起動

### 依存解決（推奨）
ワークスペースのルートで:

```bash
rosdep install --from-paths src --ignore-src -r -y
```


---

## 4. 起動方法（使い方）

通常は、用途に応じて 4つの launch のどれかを起動します。

### 4.1 開始時の挨拶（初回イベントで使ったもの）
```bash
roslaunch techrie_demo greeting_mode.launch
```

### 4.2 イベント本体（初回イベント・最終回イベントで使ったもの）
```bash
roslaunch techrie_demo event_mode.launch
```

### 4.3 日次運用（通常導入のときにつかったもの）
```bash
roslaunch techrie_demo daily_mode.launch
```

### 4.4 終了時の挨拶（最終回イベントでつかったもの）
```bash
roslaunch techrie_demo end_greeting_mode.launch
```

---

## 5. 主要トピック（初見でまず見るところ）

### 人 → ロボット（入力系）
`event_mode` では、以下の入力トピックをきっかけにロボットの行動を進行します。

- `/human/invite` … 「描いて！」などの開始要求
- `/human/show_art` … 人が作品を見せる
- `/human/praise` … 人が褒める
- `/human/pet` … なでる / 触れる

> 実際の入力デバイス（例: ボタン、Launchpad）からの変換は `human_input_bridge.py` などが担当します。

### ロボット → 表現（出力系）
- `/robot_text` (`std_msgs/String`)  
  ロボットの台詞テキスト
- `/emotion/set` (`std_msgs/String`)  
  感情ラベルの設定
- `/motion/play` (`std_msgs/String`)  
  姿勢・シーケンス・軌道の再生命令（例: `pose:...`, `seq:...`, `traj:...`）

### イベント通知
- `/interaction_events` (`techrie_demo/InteractionEvent`)  
  イベント状態の通知・監視に使うトピック  
  例: `SULK_START` などのイベントトリガ

### 日記/記録系（モードにより使用）
- 写真保存 / 日記記録系ノードが内部で利用するトピック・サービス
- 出力先は `~/.ros/techrie_demo/` 配下（後述）

---

## 6. 主要ノード（役割ベース）

以下は、初めて読むときに把握しておくと全体像がつかみやすい主要ノードです。  
（どの launch に含まれるかは各 launch ファイルを参照）

### 行動実行系
- `nodes/hw/motion_player.py`  
  `pose/seq/traj` 形式のモーション再生を担当する実行ノード

### イベント進行系
- `nodes/event/event_orchestrator.py`  
  `event_mode` の進行役（人入力・ロボ反応・自己提案・割り込み制御）

- `nodes/event/ask_and_confirm_node.py`  
  0/1入力などによる確認（OK/NG）を扱う補助ノード

### 表現・反応系
- `nodes/social/reaction_router.py`  
  発話/感情ラベル/軽いモーションなど、反応表現のハブ

### 記録系
- `nodes/logging/photo_recorder.py`  
  写真の保存

- `nodes/logging/diary_logger.py`  
  日記ログ（Markdown/DB）の保存

### 入力ブリッジ系
- `nodes/bridge/human_input_bridge.py`  
  外部入力（ボタンなど）を `/human/*` に橋渡し

---

## 7. 保存先（生成物）

このパッケージは、生成物を **source tree ではなく `~/.ros` 配下** に保存する設計です。

主な保存先:
- `~/.ros/techrie_demo/object_images/` … 画像・スナップショット
- `~/.ros/techrie_demo/diary/` … 日記（Markdown など）
- `~/.ros/techrie_demo/diary.db` … 日記DB
- `~/.ros/techrie_demo/profile/` … 実行時プロフィール状態

> これにより、リポジトリを汚さず、複数環境で扱いやすくしています。

---

## 8. 設定ファイル（GPT/Azure OpenAI など）

秘密情報を含む設定ファイルは **Git管理しません**。  
テンプレートをコピーしてローカルで設定してください。

### テンプレートから作成
```bash
cp config/gpt_api.yaml.example config/gpt_api.yaml
```

### 重要
- `config/gpt_api.yaml` は `.gitignore` 対象です
- APIキーはコミットしないでください

---

## 9. 開発の入口（どこから読むか）

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

## 10. よく使う確認コマンド（デバッグ）

### ノード一覧
```bash
rosnode list
```

### トピック一覧（human / robot / interaction を絞って見る）
```bash
rostopic list | egrep "human|robot|interaction"
```

### イベント通知を見る
```bash
rostopic echo /interaction_events
```

### ロボットの台詞を見る
```bash
rostopic echo /robot_text
```

### ノード接続を可視化
```bash
rqt_graph
```

---

## 11. トラブルシュート

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

---

## 12. 補足

- 実行時に使う外部ノード・ハードウェア（カメラ、入力装置、ロボット実機側ノード）は環境依存です
- 詳細なパラメータは各 `launch/*.launch` と各ノード内の `rospy.get_param(...)` を参照してください