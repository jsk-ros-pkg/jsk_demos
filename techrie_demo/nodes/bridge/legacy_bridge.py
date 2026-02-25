#!/usr/bin/env python3
import rospy, threading
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String, Bool

class LegacyBridge:
    def __init__(self):
        self._lock = threading.Lock()
        # 原点とスケール（m→既存系の座標系に合わせて係数調整）
        self._x = rospy.get_param("~origin_x", 0.0)
        self._y = rospy.get_param("~origin_y", 0.0)
        self._z = rospy.get_param("~origin_z", 0.0)
        self._sx = rospy.get_param("~scale_x", 1.0)
        self._sy = rospy.get_param("~scale_y", 1.0)
        self._sz = rospy.get_param("~scale_z", 1.0)
        # ソフトリミット（必要に応じて実値に）
        self._xmin = rospy.get_param("~xmin", -0.2)
        self._xmax = rospy.get_param("~xmax",  0.5)
        self._ymin = rospy.get_param("~ymin", -0.3)
        self._ymax = rospy.get_param("~ymax",  0.3)
        self._zmin = rospy.get_param("~zmin", -0.1)
        self._zmax = rospy.get_param("~zmax",  0.3)

        self.legacy_pos_pub = rospy.Publisher('/paint_position', Point, queue_size=10,latch=True)
        self.legacy_motion_pub = rospy.Publisher('/paint_motion', String, queue_size=10)

        rospy.Subscriber('/arm/move_tip', Point, self.move_tip_cb, queue_size=20)
        rospy.Service('/arm/pen_down', SetBool, self.pen_down_srv)
        rospy.Subscriber('/arm/brush_motion', String, self.brush_motion_cb, queue_size=5)

        # 任意：既存の完了通知を受ける（次ステップで使用）
        self._motion_done = False
        rospy.Subscriber('/motion_done', Bool, self.motion_done_cb)

        # 追加：原点を外からセットできるサービス
        rospy.Service('~set_origin', SetBool, self.set_origin_srv)  # Trueで(0,0,0)に、Falseは無視

        rospy.loginfo("legacy_bridge ready (origin=%.3f,%.3f,%.3f scale=%.2f,%.2f,%.2f)",
                      self._x, self._y, self._z, self._sx, self._sy, self._sz)

        p0 = Point(self._x, self._y, self._z)
        self.legacy_pos_pub.publish(p0)

    def clamp(self, v, lo, hi): return max(lo, min(hi, v))

    def move_tip_cb(self, dp: Point):
        with self._lock:
            # 相対→スケール→累積
            self._x += dp.x * self._sx
            self._y += dp.y * self._sy
            self._z += dp.z * self._sz
            # ソフトリミット
            self._x = self.clamp(self._x, self._xmin, self._xmax)
            self._y = self.clamp(self._y, self._ymin, self._ymax)
            self._z = self.clamp(self._z, self._zmin, self._zmax)
            p = Point(self._x, self._y, self._z)
        self.legacy_pos_pub.publish(p)
        rospy.loginfo_throttle(1.0, "bridge: abs(%.3f, %.3f, %.3f)", p.x, p.y, p.z)

    def pen_down_srv(self, req):
        motion = "reach" if req.data else "straight"
        self.legacy_motion_pub.publish(String(motion))
        return SetBoolResponse(success=True, message=motion)

    def brush_motion_cb(self, m: String):
        self.legacy_motion_pub.publish(m)

    def motion_done_cb(self, m: Bool):
        self._motion_done = bool(m.data)

    def set_origin_srv(self, req):
        # Trueなら原点(0,0,0)にリセット（必要なら別srvで任意値も）
        if req.data:
            with self._lock:
                self._x = self._y = self._z = 0.0
            rospy.loginfo("legacy_bridge: origin reset to (0,0,0)")
            return SetBoolResponse(True, "origin_reset")
        return SetBoolResponse(True, "noop")

def main():
    rospy.init_node("legacy_bridge")
    LegacyBridge()
    rospy.spin()

if __name__ == "__main__":
    main()

