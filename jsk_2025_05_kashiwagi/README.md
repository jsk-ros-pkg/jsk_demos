# 事前設定
config以下のudevのルールとalsaの設定を使用するPCに反映させる
```
sudo cp /path/to/jsk_2025_05_kashiwagi/config/alsa_settings/90-kashiwagi.conf /etc/alsa/conf.d/
sudo cp /path/to/jsk_2025_05_kashiwagi/config/udev/99-kashiwagi.rules /usr/lib/udev/rules.d/
```

