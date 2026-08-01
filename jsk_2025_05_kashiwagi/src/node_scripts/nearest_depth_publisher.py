#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import time

class NearestDepthPublisher:
    def __init__(self):
        rospy.init_node("nearest_depth_publisher")

        # Parameters
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.pub_topic   = rospy.get_param("~publish_topic", "/nearest_distance")
        self.center_crop_ratio = float(rospy.get_param("~center_crop_ratio", 1.0))
        self.min_range_m = rospy.get_param("~min_range_m", 0.0)
        self.max_range_m = rospy.get_param("~max_range_m", 2.0)

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.pub_topic, Float32, queue_size=10)

        # --- 追加 ---
        self.last_process_time = 0.0
        self.process_interval = 0.5  # 秒
        # --------------

        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo("Launching nearest depth publisher node ...")
        rospy.spin()

    def depth_callback(self, msg: Image):
        now = time.time()

        # --- 追加: 0.5秒以内の呼び出しはスキップ ---
        if now - self.last_process_time < self.process_interval:
            return
        self.last_process_time = now
        # --------------------------------------------------

        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth_img is None:
                self.pub.publish(Float32(data=float('nan')))
                return

            h, w = depth_img.shape[:2]
            r = np.clip(self.center_crop_ratio, 0.05, 1.0)
            if r < 1.0:
                ch, cw = int(h * r), int(w * r)
                y0 = (h - ch) // 2
                x0 = (w - cw) // 2
                depth_img = depth_img[y0:y0+ch, x0:x0+cw]

            if depth_img.dtype == np.uint16:
                depth_m = depth_img.astype(np.float32) * 0.001
            elif depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
                depth_m = depth_img.astype(np.float32)
            else:
                rospy.logwarn_throttle(2.0, f"Unsupported depth dtype: {depth_img.dtype}, publishing NaN")
                self.pub.publish(Float32(data=float('nan')))
                return

            mask = np.isfinite(depth_m)
            mask &= depth_m > 0.0
            if self.min_range_m is not None:
                mask &= depth_m >= float(self.min_range_m)
            if self.max_range_m is not None:
                mask &= depth_m <= float(self.max_range_m)

            if not np.any(mask):
                rospy.loginfo_throttle(2.0, "No valid depth found in the current frame")
                self.pub.publish(Float32(data=float('nan')))
                return

            nearest = float(np.min(depth_m[mask]))

            rospy.loginfo_throttle(0.5, f"Nearest distance: {nearest:.3f} m")
            self.pub.publish(Float32(data=nearest))

        except Exception as e:
            rospy.logerr(f"Depth processing error: {e}")
            self.pub.publish(Float32(data=float('nan')))

if __name__ == "__main__":
    try:
        NearestDepthPublisher()
    except rospy.ROSInterruptException:
        pass
