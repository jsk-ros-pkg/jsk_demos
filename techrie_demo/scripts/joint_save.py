#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
joint_save.py
- /joint_states を購読し、Enterキーで現在の関節角をキャプチャ
- 入力したポーズ名で ~/enshu_ws/src/techrie_demo/config/arm_poses.yaml に登録
- 腕は 7 関節 (larm_joint0..6, rarm_joint0..6) を対象とし、既存YAMLがあれば追記更新
"""

from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
import yaml
import os
import sys
import time
import datetime
import rospkg

pkg_root = rospkg.RosPack().get_path("techrie_demo")
YAML_PATH = os.path.join(pkg_root, "config", "arm_poses.yaml")

# デフォルトの腕ジョイント名（7関節）
DEFAULT_LEFT_JOINTS  = ['larm_joint0','larm_joint1','larm_joint2','larm_joint3','larm_joint4','larm_joint5','larm_joint6']
DEFAULT_RIGHT_JOINTS = ['rarm_joint0','rarm_joint1','rarm_joint2','rarm_joint3','rarm_joint4','rarm_joint5','rarm_joint6']

latest_js = None  # 最新のJointStateを保持

def js_cb(msg):
    global latest_js
    latest_js = msg

def load_yaml(path):
    """YAMLを読み込み。なければ新規テンプレを返す。"""
    if os.path.exists(path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}

    # joints セクションを確保
    joints = data.get('joints') or {}
    left_joints  = joints.get('left')  or DEFAULT_LEFT_JOINTS[:]
    right_joints = joints.get('right') or DEFAULT_RIGHT_JOINTS[:]

    # 7関節に強制（余分は捨て、足りなければデフォで補完）
    def fix7(names, defaults):
        names = list(names)
        # デフォルト順序で7個に合わせる（存在しなければ defaults の該当を使う）
        fixed = []
        for j in defaults:
            if j in names:
                fixed.append(j)
        # 万一少なければ defaults で埋める
        while len(fixed) < 7:
            for j in defaults:
                if j not in fixed:
                    fixed.append(j)
                if len(fixed) >= 7:
                    break
        return fixed[:7]

    left_joints  = fix7(left_joints,  DEFAULT_LEFT_JOINTS)
    right_joints = fix7(right_joints, DEFAULT_RIGHT_JOINTS)

    # poses / sequences セクション
    poses = data.get('poses') or {}
    sequences = data.get('sequences') or {}

    # 正規化して返す
    data_norm = {
        'joints': {
            'left': left_joints,
            'right': right_joints
        },
        'poses': poses,
        'sequences': sequences
    }
    return data_norm

def backup_file(path):
    """既存YAMLのバックアップを作成"""
    if not os.path.exists(path):
        return None
    ts = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    bak = path + '.bak.' + ts
    try:
        with open(path, 'r') as src, open(bak, 'w') as dst:
            dst.write(src.read())
        return bak
    except Exception as e:
        print('[WARN] backup failed: %s' % e)
        return None

def write_yaml(path, data):
    """YAMLを書き出し（keys順保持）"""
    # ディレクトリが無ければ作成
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d)
    with open(path, 'w') as f:
        yaml.safe_dump(
            data,
            f,
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=True
        )

def positions_from_jointstate(js, joint_names):
    """JointState から指定順の positions 配列を作る"""
    if js is None:
        return None
    name_to_pos = {}
    for n, p in zip(js.name, js.position):
        name_to_pos[n] = p
    pos_list = []
    missing = []
    for j in joint_names:
        if j in name_to_pos:
            pos_list.append(float(name_to_pos[j]))
        else:
            pos_list.append(0.0)
            missing.append(j)
    if missing:
        print('[WARN] missing joints in /joint_states: %s' % ', '.join(missing))
    return pos_list

def format_pos(pos):
    """見やすいよう丸め（6桁）"""
    return [round(float(x), 6) for x in pos]

def main():
    rospy.init_node('joint_pose_saver', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, js_cb)
    print('--- joint_save.py ---')
    print('Enter を押すと現在の /joint_states をキャプチャしてポーズ名を登録します。')
    print('コマンド: ":q" で終了 / ":list" で登録済みポーズ一覧 / ":joints" で対象ジョイントを表示\n')

    data = load_yaml(YAML_PATH)
    left_joints  = data['joints']['left']
    right_joints = data['joints']['right']

    # 起動直後は JointState が入るまで待機
    rate = rospy.Rate(10)
    print('Waiting for /joint_states ...')
    while not rospy.is_shutdown() and latest_js is None:
        rate.sleep()
    if rospy.is_shutdown():
        return
    print('OK: /joint_states 受信開始')

    while not rospy.is_shutdown():
        try:
            user = raw_input('Enter で記録、":q" で終了 > ') if sys.version_info[0] < 3 else input('Enter で記録、":q" で終了 > ')
        except (EOFError, KeyboardInterrupt):
            print('\nBye.')
            break

        if user.strip() == ':q':
            print('Bye.')
            break
        if user.strip() == ':list':
            print('登録済みポーズ: %s' % (', '.join(sorted(data['poses'].keys())) if data['poses'] else '(なし)'))
            continue
        if user.strip() == ':joints':
            print('left joints : %s' % left_joints)
            print('right joints: %s' % right_joints)
            continue

        # 現在の関節角を取得
        js = latest_js
        left_pos  = positions_from_jointstate(js, left_joints)
        right_pos = positions_from_jointstate(js, right_joints)
        if left_pos is None or right_pos is None:
            print('[ERROR] /joint_states が未受信です。')
            continue

        left_pos  = format_pos(left_pos)
        right_pos = format_pos(right_pos)

        print('\n--- 現在値（左7/右7）---')
        print('L:', left_pos)
        print('R:', right_pos)

        # ポーズ名入力
        try:
            pose_name = raw_input('登録するポーズ名を入力してください（空でキャンセル）: ') if sys.version_info[0] < 3 else input('登録するポーズ名を入力してください（空でキャンセル）: ')
        except (EOFError, KeyboardInterrupt):
            print('\nキャンセルしました。')
            continue

        pose_name = pose_name.strip()
        if not pose_name:
            print('キャンセルしました。\n')
            continue

        # dur 入力（任意）
        try:
            dur_in = raw_input('dur（秒, 未入力で1.0）: ') if sys.version_info[0] < 3 else input('dur（秒, 未入力で1.0）: ')
        except (EOFError, KeyboardInterrupt):
            dur_in = ''
        try:
            dur = float(dur_in) if dur_in.strip() != '' else 1.0
        except ValueError:
            print('[WARN] dur が数値でないため 1.0 を使用します。')
            dur = 1.0

        # 既存YAMLを最新読み込み（並行編集への耐性）
        data = load_yaml(YAML_PATH)
        data['poses'][pose_name] = {
            'left':  left_pos,
            'right': right_pos,
            'dur': float(round(dur, 3))
        }

        # バックアップ & 書き込み
        bak = backup_file(YAML_PATH)
        write_yaml(YAML_PATH, data)

        print('\n[OK] 追加/更新しました: %s' % YAML_PATH)
        if bak:
            print('      旧ファイルのバックアップ: %s' % bak)
        print('      poses["%s"] = {left:[7], right:[7], dur:%s}\n' % (pose_name, dur))

    # 終了
    rospy.signal_shutdown('user exit')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
