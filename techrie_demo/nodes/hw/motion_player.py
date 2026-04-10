#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, yaml
import rospy, actionlib, rospkg
import json
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# InteractionEvent.msg（存在しない環境では String にフォールバック）
try:
    from techrie_demo.msg import InteractionEvent
    HAS_IE = True
except Exception:
    HAS_IE = False
    InteractionEvent = None

class MotionPlayer:
    def __init__(self):
        rospy.init_node("motion_player")

        # ===== Parameters =====
        default_base = rospkg.RosPack().get_path("techrie_demo")
        base = os.path.expanduser(rospy.get_param("~motion_base_dir", default_base))
        self.pose_file   = rospy.get_param("~pose_file",  os.path.join(base, "config/arm_poses.yaml"))
        self.motion_dir  = rospy.get_param("~motion_dir", os.path.join(base, "motions"))
        self.larm_action = rospy.get_param("~larm_action", "/larm_controller/follow_joint_trajectory")
        self.rarm_action = rospy.get_param("~rarm_action", "/rarm_controller/follow_joint_trajectory")
        self.head_action = rospy.get_param("~head_action", "/head_controller/follow_joint_trajectory")
        self.min_dt      = float(rospy.get_param("~min_dt", 0.02))
        self.start_offset = float(rospy.get_param("~start_offset", 0.05))  # 50ms 先で開始

        # ===== Action clients =====
        self.head_ac = actionlib.SimpleActionClient(self.head_action, FollowJointTrajectoryAction)
        self.larm_ac = actionlib.SimpleActionClient(self.larm_action, FollowJointTrajectoryAction)
        self.rarm_ac = actionlib.SimpleActionClient(self.rarm_action, FollowJointTrajectoryAction)
        rospy.loginfo("motion_player: waiting controllers ...")
        self.head_ac.wait_for_server()
        self.larm_ac.wait_for_server()
        self.rarm_ac.wait_for_server()
        rospy.loginfo("motion_player: controllers ready.")

        # ===== Event publisher =====
        if HAS_IE:
            self.ie_pub = rospy.Publisher('/interaction/event', InteractionEvent, queue_size=10)
        else:
            self.ie_pub = rospy.Publisher('/interaction/event_fallback', String, queue_size=10)

        # ===== Load YAML poses/sequences =====
        self._load_poses()

        # ===== Subscriber =====
        self.sub = rospy.Subscriber("/motion/play", String, self.cb_play, queue_size=50)

        rospy.loginfo("pose_file=%s", self.pose_file)
        rospy.loginfo("motion_dir=%s", self.motion_dir)
        rospy.loginfo("poses=%s", list(self.poses.keys()))
        rospy.loginfo("seqs=%s", list(self.sequences.keys()))
        rospy.loginfo("ready.")

    # ---------- utils ----------
    def _emit(self, event_type, meta=None, intensity=1.0, target_id="", actor_id="motion_player"):
        if HAS_IE:
            msg = InteractionEvent()
            msg.stamp = rospy.Time.now()
            msg.event_type = str(event_type)
            msg.actor_id = str(actor_id)
            msg.target_id = str(target_id)
            msg.intensity = float(intensity)
            msg.meta_json = json.dumps(meta or {}, ensure_ascii=False)
            self.ie_pub.publish(msg)
        else:
            # 互換フォーマット（paint_executor 側で拾える）
            self.ie_pub.publish("[InteractionEvent] " + json.dumps(
                {"event_type": event_type, "meta": (meta or {})}, ensure_ascii=False))

    def _load_poses(self):
        # 既定のジョイント順（必要に応じて YAML で上書き）
        self.l_joints = ['larm_joint0','larm_joint1','larm_joint2','larm_joint3','larm_joint4','larm_joint5','larm_joint6']
        self.r_joints = ['rarm_joint0','rarm_joint1','rarm_joint2','rarm_joint3','rarm_joint4','rarm_joint5','rarm_joint6']
        self.h_joints = ['head_joint0','head_joint1']
        self.poses = {}
        self.sequences = {}

        if not os.path.exists(self.pose_file):
            rospy.logwarn("pose_file not found: %s", self.pose_file); return

        with open(self.pose_file, 'r') as f:
            y = yaml.safe_load(f) or {}

        jnts = y.get('joints') or {}
        if 'left'  in jnts: self.l_joints = list(jnts['left'])[:len(self.l_joints)]
        if 'right' in jnts: self.r_joints = list(jnts['right'])[:len(self.r_joints)]
        if 'head'  in jnts: self.h_joints = list(jnts['head'])[:len(self.h_joints)]

        self.poses = y.get('poses') or {}

        raw_seqs = y.get('sequences') or {}
        for k, v in raw_seqs.items():
            # sequences は ["pose名", ...] もしくは [{"pose": "xxx"}, ...] の両方を許容
            if isinstance(v, dict) and 'steps' in v:
                steps = list(v['steps'])
            elif isinstance(v, list):
                steps = list(v)
            else:
                steps = []
            # {"pose": "xxx"} を "xxx" に正規化
            norm = []
            for s in steps:
                if isinstance(s, dict) and 'pose' in s:
                    norm.append(str(s['pose']))
                else:
                    norm.append(str(s))
            self.sequences[k] = norm

    def _send_point(self, names, positions, dur, ac):
        pos = list(positions)[:len(names)]
        if len(pos) < len(names):
            pos += [0.0]*(len(names)-len(pos))
        jt = JointTrajectory(); jt.joint_names = names
        jt.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.start_offset)
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.time_from_start = rospy.Duration.from_sec(float(dur))
        jt.points = [pt]
        goal = FollowJointTrajectoryGoal(trajectory=jt)
        ac.send_goal(goal)

    def _normalize_times(self, pts):
        """t を 0 始まり＆非減少に正規化。最小Δt = self.min_dt"""
        if not pts: return []
        ts = [float(p.get('t', 0.0)) for p in pts]
        t0 = ts[0]
        ts = [t - t0 for t in ts]
        norm = []
        prev = -self.min_dt
        for t in ts:
            if t <= prev: t = prev + self.min_dt
            norm.append(round(t, 4)); prev = t
        out = []
        for p, t in zip(pts, norm):
            q = dict(p); q['t'] = t; out.append(q)
        return out

    # ---------- players ----------
    def play_pose(self, name):
        p = self.poses.get(name)
        if not p:
            rospy.logwarn("pose not found: %s", name); return False
        dur = float(p.get('dur', 1.0))
        clients = []
        if 'left' in p:
            self._send_point(self.l_joints, p['left'], dur, self.larm_ac)
            clients.append(self.larm_ac)
        if 'right' in p:
            self._send_point(self.r_joints, p['right'], dur, self.rarm_ac)
            clients.append(self.rarm_ac)
        if 'head' in p:
            self._send_point(self.h_joints, p['head'], dur, self.head_ac)
            clients.append(self.head_ac)

        ok_all = True
        for ac in clients:
            ok_all = ok_all and ac.wait_for_result(rospy.Duration.from_sec(dur + 5.0))
        return ok_all

    def play_pose_with_dur(self, name, dur):
        p = self.poses.get(name)
        if not p:
            rospy.logwarn("pose not found: %s", name); return False
        try:
            dur = float(dur)
        except Exception:
            rospy.logwarn("invalid dur for pose %s; fallback to YAML", name)
            return self.play_pose(name)

        clients = []
        # 例: 左右腕・頭など、既存のplay_poseと同様の送り方でdurのみ上書き
        if 'left' in p:
            self._send_point(self.l_joints, p['left'], dur, self.larm_ac); clients.append(self.larm_ac)
        if 'right' in p:
            self._send_point(self.r_joints, p['right'], dur, self.rarm_ac); clients.append(self.rarm_ac)
        if 'head' in p:
            self._send_point(self.h_joints, p['head'], dur, self.head_ac); clients.append(self.head_ac)

        ok_all = True
        # 送信先アクションの完了待ち。+5秒など安全マージンは既存実装に合わせる
        for ac in clients:
            ok_all = ok_all and ac.wait_for_result(rospy.Duration.from_sec(dur + 5.0))
        return ok_all


    def play_sequence(self, name):
        steps = self.sequences.get(name)
        if not steps:
            rospy.logwarn("sequence not found: %s", name); return False
        ok_all = True
        for pose_name in steps:
            ok = self.play_pose(pose_name)
            ok_all = ok_all and ok
        return ok_all

    def play_grip(self, side='both', value=1.0, dur=0.5):
        rospy.logwarn('grip placeholder: side=%s value=%.3f dur=%.2f (not implemented)', side, value, dur)
        return True

    def play_traj(self, name):
        # JSON -> left/right/head の JointTrajectory
        path = os.path.join(self.motion_dir, name + '.json')
        if not os.path.exists(path):
            rospy.logwarn('traj file not found: %s', path); return False
        try:
            with open(path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            rospy.logwarn('traj json load error: %s', e); return False

        pts = data.get('points', [])
        if not pts:
            rospy.logwarn('traj points empty: %s', path); return False
        pts = self._normalize_times(pts)

        # ジョイント順（JSONに joints があれば尊重、無ければ現在の内部順）
        src_l = (data.get('joints') or {}).get('left',  self.l_joints)
        src_r = (data.get('joints') or {}).get('right', self.r_joints)
        src_h = (data.get('joints') or {}).get('head',  self.h_joints)

        def reorder(src_names, dst_names, values):
            name_to_idx = {n:i for i,n in enumerate(src_names)}
            out = []
            for n in dst_names:
                out.append(float(values[name_to_idx[n]]) if n in name_to_idx else 0.0)
            return out

        def build_if_present(dst_names, key, src_names):
            # key（left/right/head）が 1 度でも出てくる場合のみ生成
            present = any((key in p) for p in pts)
            if not present: return None
            jt = JointTrajectory(); jt.joint_names = dst_names
            jt.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(self.start_offset)
            for p in pts:
                arr = list(p.get(key, [0.0]*len(dst_names)))
                if len(arr) != len(dst_names) or src_names != dst_names:
                    arr = reorder(src_names, dst_names, arr)
                jtp = JointTrajectoryPoint()
                jtp.positions = arr[:len(dst_names)]
                jtp.time_from_start = rospy.Duration.from_sec(float(p['t']))
                jt.points.append(jtp)
            return jt

        jt_l = build_if_present(self.l_joints, 'left',  src_l)
        jt_r = build_if_present(self.r_joints, 'right', src_r)
        jt_h = build_if_present(self.h_joints, 'head',  src_h)

        total_t = float(pts[-1]['t'])
        rospy.loginfo("traj '%s': points=%d, total=%.3fs", name, len(pts), total_t)

        clients = []
        if jt_l:
            self.larm_ac.send_goal(FollowJointTrajectoryGoal(trajectory=jt_l))
            clients.append(self.larm_ac)
        if jt_r:
            self.rarm_ac.send_goal(FollowJointTrajectoryGoal(trajectory=jt_r))
            clients.append(self.rarm_ac)
        if jt_h:
            self.head_ac.send_goal(FollowJointTrajectoryGoal(trajectory=jt_h))
            clients.append(self.head_ac)

        ok_all = True
        for ac in clients:
            ok_all = ok_all and ac.wait_for_result(rospy.Duration.from_sec(total_t + 5.0))
        return ok_all

    # ---------- callback ----------
    def cb_play(self, msg):
        raw = (msg.data or '').strip()
        if not raw:
            return
    
        ok = False                 # ← 先に初期化しておく（UnboundLocalError防止）
        motion_id = raw
        meta = {}

        try:
            # JSON 指定にも対応：{"id":"pose:reset", "value":..., "dur":...} など
            if raw.startswith('{'):
                obj = json.loads(raw)
                motion_id = obj.get('id', '')
                meta = obj or {}

            if motion_id == 'nod':
                ok = self.play_sequence('nod') if 'nod' in self.sequences else False

            elif motion_id == 'shake':
                ok = self.play_sequence('shake') if 'shake' in self.sequences else False

            elif motion_id.startswith('pose:'):
                name = motion_id.split(':', 1)[1]

                # JSONでdurが来ていれば、その回だけdurationを上書き
                if isinstance(meta, dict) and 'dur' in meta:
                    try:
                        d = float(meta.get('dur'))
                        ok = self.play_pose_with_dur(name, d)
                    except Exception:
                        rospy.logwarn("invalid dur in JSON for pose:%s; fallback to YAML", name)
                        ok = self.play_pose(name)
                else:
                    # dur が無い場合は通常の pose 再生
                    ok = self.play_pose(name)

            elif motion_id.startswith('seq:'):
                ok = self.play_sequence(motion_id.split(':', 1)[1])

            elif motion_id.startswith('traj:'):
                ok = self.play_traj(motion_id.split(':', 1)[1])

            elif motion_id.startswith('grip:'):
                side = motion_id.split(':', 1)[1]
                val = float(meta.get('value', 1.0))
                dur = float(meta.get('dur', 0.5))
                ok = self.play_grip(side, val, dur)

            else:
                rospy.logwarn("motion_player: unknown motion '%s'", motion_id)
                ok = False

        except Exception as e:
            rospy.logerr("motion_player: exception on '%s': %s", motion_id, e)
            ok = False

        # paint_executor などが待てるよう、送られた motion_id をそのまま返す
        self._emit("MOTION_END", {"motion": motion_id, "success": bool(ok)})


def main():
    MotionPlayer()
    rospy.spin()

if __name__ == "__main__":
    main()
