#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, os, json, time, random, glob
from std_msgs.msg import String

class JitterPlayer:
    def __init__(self):
        rospy.init_node("traj_jitter_player")

        base = os.path.expanduser('~/enshu_ws/src/techrie_demo')
        self.motion_dir = rospy.get_param("~motion_dir", os.path.join(base, "motions"))
        self.out_subdir = rospy.get_param("~out_subdir", "_jit")
        self.out_dir    = os.path.join(self.motion_dir, self.out_subdir)

        self.jitter_prefixes = rospy.get_param("~jitter_prefixes", ["draw/"])
        self.pos_sigma  = float(rospy.get_param("~pos_sigma_rad", 0.01))
        self.pos_clip   = float(rospy.get_param("~pos_clip_rad", 0.03))
        self.t_sigma    = float(rospy.get_param("~t_sigma_sec", 0.0))
        self.min_dt     = float(rospy.get_param("~min_dt", 0.04))
        self.max_cache  = int(rospy.get_param("~max_cache_files", 200))

        os.makedirs(self.out_dir, exist_ok=True)
        self.pub_motion = rospy.Publisher("/motion/play", String, queue_size=10)
        rospy.Subscriber("/traj_play", String, self.on_cmd, queue_size=50)
        rospy.loginfo("traj_jitter_player: relay /traj_play -> /motion/play  dir=%s", self.out_dir)

    def _gauss_clip(self, s, clip):
        if s <= 0.0: return 0.0
        x = random.gauss(0.0, s)
        return max(-clip, min(clip, x))

    def _normalize_times(self, pts):
        if not pts: return []
        out, prev = [], 0.0
        for i, p in enumerate(pts):
            t = float(p.get('t', 0.0))
            if i == 0: t = 0.0
            t = t + self._gauss_clip(self.t_sigma, self.t_sigma*3)
            if i > 0: t = max(prev + self.min_dt, t)
            prev = t
            q = dict(p); q['t'] = round(t, 4); out.append(q)
        return out

    def _jitter(self, data):
        out_pts = []
        for p in data.get('points', []):
            q = dict(p)
            for key in ('left','right','head'):
                if key in q:
                    arr = list(q[key])
                    for i in range(len(arr)):
                        arr[i] = float(arr[i]) + self._gauss_clip(self.pos_sigma, self.pos_clip)
                    q[key] = arr
            out_pts.append(q)
        d = dict(data); d['points'] = self._normalize_times(out_pts)
        return d

    def _save_and_publish(self, name, d):
        stamp = "%d_%04d" % (int(time.time()*1000), random.randint(0,9999))
        safe  = name.replace("/","_")
        out_name = f"{safe}__{stamp}.json"
        path = os.path.join(self.out_dir, out_name)
        with open(path, "w") as f:
            json.dump(d, f, ensure_ascii=False)

        files = sorted(glob.glob(os.path.join(self.out_dir,"*.json")))
        if len(files) > self.max_cache:
            for f in files[:-self.max_cache]:
                try: os.remove(f)
                except: pass

        relay = f"traj:{self.out_subdir}/{out_name[:-5]}"
        self.pub_motion.publish(String(relay))
        rospy.loginfo("jit: %s -> %s", name, relay)

    def _should_jitter(self, name):
        if name.startswith(self.out_subdir + "/"):   # 既に _jit のものは素通し
            return False
        return any(name.startswith(pref) for pref in self.jitter_prefixes)

    def on_cmd(self, msg):
        raw = (msg.data or "").strip()
        name = raw.split(":",1)[1] if raw.startswith("traj:") else raw
        if not self._should_jitter(name):
            self.pub_motion.publish(String("traj:" + name))  # reach_paint 等は素通し
            rospy.loginfo("passthrough: %s", name)
            return

        path = os.path.join(self.motion_dir, name + ".json")
        if not os.path.exists(path):
            rospy.logwarn("base not found: %s", path); return
        try:
            with open(path, "r") as f:
                base = json.load(f)
        except Exception as e:
            rospy.logwarn("json load err: %s", e); return

        self._save_and_publish(name, self._jitter(base))

if __name__=="__main__":
    JitterPlayer(); rospy.spin()
