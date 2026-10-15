#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import time

class QRReader:
    def __init__(self):
        rospy.init_node('qr_reader')
        rospy.sleep(1.0)
        self.current_kashiwagi_state = "unknown"
        self.bridge = CvBridge()

        # --- 追加: 処理間隔（秒）と最後の処理時刻 ---
        self.process_interval = float(rospy.get_param("~process_interval", 0.5))
        self._last_process_time = 0.0
        # ------------------------------------------------

        # 最新フレームだけ処理するため queue_size=1
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback,
                                          queue_size=1, buff_size=2**24)
        self.state_sub = rospy.Subscriber('/kashiwagi_state', String, self.state_callback, queue_size=1)
        self.text_pub = rospy.Publisher('/qr_data', String, queue_size=10)
        self.pos_pub = rospy.Publisher('/qr_position', Point, queue_size=10)
        rospy.loginfo("Launching QR code reader node ....")
        rospy.spin()

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data

    def image_callback(self, msg):
        # 状態が一致しないときは処理しない
        # if self.current_kashiwagi_state != "talking_game:listening_turn":
        #     return

        # --- 追加: 0.5秒以内の呼び出しはスキップ ---
        now = time.time()
        if (now - self._last_process_time) < self.process_interval:
            return
        self._last_process_time = now
        # ------------------------------------------------

        # ここから実処理
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qr_codes = decode(frame)

        for obj in qr_codes:
            data = obj.data.decode('utf-8')
            rospy.loginfo_throttle(0.5, f"QR data : {data}")
            self.text_pub.publish(data)

            pts = obj.polygon
            if len(pts) == 4:
                cx = sum([p.x for p in pts]) / 4.0
                cy = sum([p.y for p in pts]) / 4.0
                rospy.loginfo_throttle(0.5, f"QR position: x={cx:.1f}, y={cy:.1f}")

                point = Point(x=cx, y=cy, z=0.0)
                self.pos_pub.publish(point)

if __name__ == '__main__':
    try:
        QRReader()
    except rospy.ROSInterruptException:
        pass
