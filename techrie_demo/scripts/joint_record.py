#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, datetime, select, sys
import rospy
from sensor_msgs.msg import JointState
import rospkg

pkg_root = rospkg.RosPack().get_path("techrie_demo")
OUT_DIR = os.path.join(pkg_root, "motions")
LEFT  = ['larm_joint0','larm_joint1','larm_joint2','larm_joint3','larm_joint4','larm_joint5','larm_joint6']
RIGHT = ['rarm_joint0','rarm_joint1','rarm_joint2','rarm_joint3','rarm_joint4','rarm_joint5','rarm_joint6']

class Recorder:
    def __init__(self, rate_hz=50.0, min_dt=0.02):
        self.latest = None
        self.points = []   # [{t, left:[7], right:[7]}]
        self.t0 = None     # rospy.Time
        self.prev_t = 0.0
        self.rate = rospy.Rate(rate_hz)
        self.min_dt = float(min_dt)
        rospy.Subscriber('/joint_states', JointState, self.cb)

    def cb(self, msg):
        self.latest = msg

    def _pick(self, names, js):
        name_to_idx = {n:i for i,n in enumerate(js.name)}
        return [float(js.position[name_to_idx[n]]) if n in name_to_idx else 0.0 for n in names]

    def _stdin_ready(self, timeout=0.0):
        return sys.stdin in select.select([sys.stdin], [], [], timeout)[0]

    def _save_now(self):
        if not self.points:
            print('（記録データが空のため保存しません）'); return
        if not os.path.exists(OUT_DIR):
            os.makedirs(OUT_DIR)
        ts = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        try:
            name = input('保存名（例: bye_bye / 未入力で motion_'+ts+'）: ').strip() or ('motion_'+ts)
        except (EOFError, KeyboardInterrupt):
            name = 'motion_'+ts
        out_path = os.path.join(OUT_DIR, name + '.json')
        data = {
            'version': 1,
            'joints': {'left': LEFT, 'right': RIGHT},
            'meta': {'created_at': ts, 'source': 'joint_record.py', 'rate_hz': 1.0/self.min_dt},
            'points': self.points
        }
        with open(out_path, 'w') as f:
            json.dump(data, f, indent=2)
        print('[OK] 保存:', out_path)

    def loop(self):
        print('Enter: 記録開始/一時停止（停止時に保存します） / :q で終了')
        print('Waiting /joint_states ...')
        while not rospy.is_shutdown() and self.latest is None:
            self.rate.sleep()
        print('OK: /joint_states 受信中')

        recording = False
        try:
            while not rospy.is_shutdown():
                if not recording:
                    cmd = input('[待機] Enterで記録開始 / :q で終了 > ').strip()
                    if cmd == ':q':
                        print('Bye.'); break
                    # start
                    self.points = []
                    self.t0 = rospy.Time.now()
                    self.prev_t = 0.0
                    recording = True
                    print('Recording... （もう一度Enterで停止→保存）')
                else:
                    if self.latest:
                        # ローカル時間基準の単調増加 t
                        t = (rospy.Time.now() - self.t0).to_sec()
                        if t <= self.prev_t:
                            t = self.prev_t + self.min_dt
                        self.prev_t = t
                        self.points.append({
                            't': round(float(t), 4),
                            'left':  self._pick(LEFT,  self.latest),
                            'right': self._pick(RIGHT, self.latest),
                        })
                    if self._stdin_ready(0.0):
                        sys.stdin.readline()
                        recording = False
                        print('Pause. 保存します…')
                        self._save_now()
                    self.rate.sleep()
        except KeyboardInterrupt:
            print('\nCtrl+C 受信。必要なら最後の録画を保存します。')
            self._save_now()

def main():
    rospy.init_node('joint_recorder')
    Recorder(rate_hz=50.0, min_dt=0.02).loop()

if __name__ == '__main__':
    main()
