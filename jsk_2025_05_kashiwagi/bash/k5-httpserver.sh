#!/usr/bin/env bash

deactivate
PKG_PATH=$(rospack find jsk_2025_05_kashiwagi)
WEB_DIR="$PKG_PATH/src/node_scripts/propose_game/ui"

roslaunch rosbridge_server rosbridge_websocket.launch &
python3 -m http.server 8000 --directory "$WEB_DIR"
