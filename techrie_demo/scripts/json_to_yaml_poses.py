#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, yaml, sys

MOTION_DIR = os.path.expanduser('~/enshu_ws/src/techrie_demo/motions')
YAML_PATH  = os.path.expanduser('~/enshu_ws/src/techrie_demo/config/arm_poses.yaml')

def main(name, step_dur=0.04):
    jpath = os.path.join(MOTION_DIR, name + '.json')
    if not os.path.exists(jpath):
        print('not found:', jpath); sys.exit(1)
    with open(jpath, 'r') as f: data = json.load(f)
    y = {}
    if os.path.exists(YAML_PATH):
        with open(YAML_PATH, 'r') as f: y = yaml.safe_load(f) or {}
    y.setdefault('joints', {'left': data['joints']['left'], 'right': data['joints']['right']})
    y.setdefault('poses', {})
    y.setdefault('sequences', {})
    steps = []
    for i,p in enumerate(data['points']):
        pose_name = f'{name}_{i:04d}'
        y['poses'][pose_name] = {'left': p['left'], 'right': p['right'], 'dur': float(step_dur)}
        steps.append(pose_name)
    y['sequences'][name] = {'steps': steps}
    with open(YAML_PATH, 'w') as f:
        yaml.safe_dump(y, f, sort_keys=False, allow_unicode=True)
    print('[OK] exported to YAML:', YAML_PATH, '(sequence:', name, ')')

if __name__ == '__main__':
    if len(sys.argv)<2:
        print('usage: json_to_yaml_poses.py NAME [step_dur]')
        sys.exit(1)
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv)>2 else 0.04)
