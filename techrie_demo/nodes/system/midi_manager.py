#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, subprocess, shlex, re, time

# ★環境に合わせて“部分一致”で指定（aconnect -l の文字列に合わせる）
DEFAULT_IN_NAME  = "Launchpad X LPX MIDI In"  # デバイス側の受け口（PC→Launchpad）
DEFAULT_APP_NAME = "RtMidi output"           # アプリ側の出力（App→外部）例として掲載

def run(cmd, timeout=3):
    try:
        out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT, timeout=timeout)
        return out.decode(errors="ignore")
    except Exception as e:
        rospy.logwarn("midi_manager: %s failed: %s", cmd, e)
        return ""

def aconnect_reset():
    rospy.loginfo("midi_manager: aconnect -x")
    run("aconnect -x", timeout=5)
    time.sleep(0.2)

def parse_clients(text):
    clients = []
    cur_client = None
    for line in text.splitlines():
        m = re.match(r"client\s+(\d+):\s+'([^']+)'", line.strip())
        if m:
            cur_client = (int(m.group(1)), m.group(2))
            continue
        m = re.match(r"\s*(\d+)\s+'([^']+)'", line.strip())
        if m and cur_client:
            clients.append({
                "client": cur_client[0],
                "client_name": cur_client[1],
                "port": int(m.group(1)),
                "port_name": m.group(2),
            })
    return clients

def find_port(clients, name_substr):
    s = name_substr.lower()
    for c in clients:
        if s in c["client_name"].lower() or s in c["port_name"].lower():
            return c["client"], c["port"], c["client_name"], c["port_name"]
    return None

def ensure_connection(src, dst):
    cmd = f"aconnect {src[0]}:{src[1]} {dst[0]}:{dst[1]}"
    out = run(cmd)
    rospy.loginfo("midi_manager: connect %s:%s -> %s:%s (%s)", src[0], src[1], dst[0], dst[1], out.strip())

def on_shutdown():
    # 終了時の固着対策
    aconnect_reset()

def main():
    rospy.init_node("midi_manager", anonymous=False)
    in_name  = rospy.get_param("~in_name",  DEFAULT_IN_NAME)
    app_name = rospy.get_param("~app_name", DEFAULT_APP_NAME)
    do_reset_on_start = rospy.get_param("~disconnect_on_start", True)
    poll_hz = float(rospy.get_param("~poll_hz", 1.0))

    rospy.on_shutdown(on_shutdown)
    rospy.loginfo("midi_manager: target in='%s', app='%s', poll=%.2fHz", in_name, app_name, poll_hz)

    if do_reset_on_start:
        aconnect_reset()

    last_key = None
    rate = rospy.Rate(poll_hz)

    while not rospy.is_shutdown():
        txt = run("aconnect -l")
        clients = parse_clients(txt)

        # ▼必要に応じて、Launchpad→アプリ、アプリ→Launchpad の両方向を張りたい場合は
        #   それぞれ src/dst を決めて2本 ensure_connection() してください。
        #   ここでは例として「アプリ出力→LaunchpadのMIDI In」を張っています。
        app = find_port(clients, app_name)
        lpx_in = find_port(clients, in_name)

        if app and lpx_in:
            key = (app[0], lpx_in[0])
            if key != last_key:
                rospy.loginfo("midi_manager: ports found app=%s:%s(%s/%s) -> lpx=%s:%s(%s/%s)",
                              app[0], app[1], app[2], app[3],
                              lpx_in[0], lpx_in[1], lpx_in[2], lpx_in[3])
                # 念のため既存接続を一度クリアしてから接続
                aconnect_reset()
                ensure_connection((app[0], app[1]), (lpx_in[0], lpx_in[1]))
                last_key = key

        rate.sleep()

if __name__ == "__main__":
    main()
