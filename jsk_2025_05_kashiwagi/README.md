# 環境構築 (2026年4月暫定)

## 1. パフォーマンスとクロックの最大化 (reComputer mini J4012用)
reComputer mini J4012を使う場合、電力を最大まで使うモードにし、クロックも最大にする（そのままだと4コアだけど、これで8コアまであげられる）。

```bash
sudo nvpmodel -q
sudo nvpmodel -m 0

sudo tee /etc/systemd/system/jetson_clocks.service >/dev/null <<'EOF'
[Unit]
Description=Jetson Clocks Startup
After=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable jetson_clocks.service
sudo reboot
```

## 2. リポジトリのクローン (`vcs import`)
ワークスペース（`ws`）の直下に以下の内容で `repos.yaml` を置く。

```yaml
repositories:
  jsk_demos:
    type: git
    url: git@github.com:sawada10/jsk_demos.git
    version: kashiwagi-call-person-name
  jsk_3rdparty:
    type: git
    url: git@github.com:sawada10/jsk_3rdparty.git
    version: kashiwagi
  audio_common:
    type: git
    url: https://github.com/iory/audio_common.git
    version: publish-playback-audio
  kashiwagi_kuromitsu:
    type: git
    url: https://github.com/a-ichikura/kashiwagi_kuromitsu.git
    version: kashiwagi
  realsense-ros:
    type: git
    url: https://github.com/IntelRealSense/realsense-ros.git
    version: ros1-legacy
  rcb4:
    type: git
    url: git@github.com:iory/rcb4.git
    version: v0.0.8
  riberry:
    type: git
    url: git@github.com:iory/riberry.git
    version: main
  modular_robot_model_zoo:
    type: git
    url: git@gitlab.jsk.imi.i.u-tokyo.ac.jp:sawada1/modular_robot_model_zoo.git
    version: add-kashiwagi-kuromitsu-models
```

ファイル配置後、以下のコマンドを実行してソースコードをインポートする。
```bash
vcs import src < repos.yaml
```

## 3. デバイス名の設定
ALSAとudevのルールを設定して反映させる。（※パスは実際の環境に合わせて読み替えること）

```bash
sudo cp /path/to/jsk_2025_05_kashiwagi/config/alsa_setting/90-kashiwagi.conf /etc/alsa/conf.d/
sudo cp /path/to/jsk_2025_05_kashiwagi/config/udev/99-kashiwagi.rules /usr/lib/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
また、以下はrespeakerの設定
```bash
sudo cp -f /path/to/respeaker_ros/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl restart udev
```

## 4. 環境変数の設定
`~/.bashrc` にAPIキーなどの設定を書く。

```bash
export AZURE_OPENAI_KEY="xxxxxxxxxxxxxxxxxxxxxxx"
export AZURE_OPENAI_ENDPOINT="https://xxxxxxxxxxxxxxxxxx"

# VOICEVOXの設定
export VOICEVOX_DEFAULT_SPEAKER_ID=6 # デフォルトのスピーカーID。好みの音声に変更可能
export VOICEVOX_TEXTTOSPEECH_URL=10.10.10.10 # 別PCのDockerでVOICEVOXを動かす場合はそのIPを記述
export VOICEVOX_TEXTTOSPEECH_PORT=50021
export ROS_VOICEVOX_TEXTTOSPEECH_CACHE_ENABLED=True
```

## 5. データのダウンロード
データ管理の都合上、git管理から外しているデータをダウンロードする。

* **talking_game.tsv**: [Google Driveリンク](https://drive.google.com/file/d/1y07vDVeBMpY2uMc6opYEtDIms2K-XFyD/view?usp=sharing)
  * *(※必要な場合はアクセス権をリクエストする)*


## 6. ライブラリ等のインストール
pipをする前に仮想環境を作りactivateする。

```bash
source /path/to/jsk_2025_05_kashiwagi/.venv/bin/activate
cd /path/to/jsk_2025_05_kashiwagi
pip install -r requirements.txt
```

> **Note**: `requirements.txt` は暫定版で足りないものがある可能性あり（2026年4月現在）。

## 7. パッケージのビルド
必要なパッケージ（`voicevox`, `eye_display`, `jsk_2023_12_codesign`, `jsk_2025_05_kashiwagi`, `realsense2_camera`, `semi2024` など）をbuildする。

### ビルド時の注意点

* **voicevox のビルド**
  voicevoxのbuildは落ちることがあるので、以下で使用するコア数を制限する。
  ```bash
  catkin build voicevox -j3 --cmake-args
  ```

* **kxr_controller のビルド**
  kxr_controllerはそのままbuildすると通らないので、以下のオプションをつける。
  ```bash
  catkin build kxr_controller --cmake-args -DUSE_VIRTUALENV=OFF -DPYTHON_EXECUTABLE=$(which python)
  ```

* **依存パッケージの手動インストール**
  `package.xml` から入らないことがある（原因不明）ので、その場合は手動で入れる。
  ```bash
  sudo apt install ros-noetic-joint-trajectory-controller
  ```
