#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import time
import math

class QRDepthReader:
    def __init__(self):
        rospy.init_node("qr_depth_reader")
        self.bridge = CvBridge()
        self.latest_px = None
        self.last_seen_time = None
        self.qr_timeout = 1.0

        # Subscribers
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.pos_sub = rospy.Subscriber("/qr_position", Point, self.position_callback)

        # Publisher
        self.distance_pub = rospy.Publisher("/qr_distance", Float32, queue_size=10)

        rospy.loginfo("Launching QR code depth reader node ....")
        rospy.spin()

    def position_callback(self, msg):
        self.latest_px = (int(msg.x), int(msg.y))
        self.last_seen_time = time.time()

    def depth_callback(self, msg):
        now = time.time()

        if self.last_seen_time is None or now - self.last_seen_time > self.qr_timeout:
            # Cannot find QR for qr_timeout [seconds]
            rospy.loginfo("Cannot find QR code now")
            self.distance_pub.publish(Float32(data=float('nan')))
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # 16UC1
            x, y = self.latest_px

            if y >= depth_image.shape[0] or x >= depth_image.shape[1]:
                # Cannot find QR code within depth image
                rospy.logwarn("QR code is out of depth image")
                self.distance_pub.publish(Float32(data=float('nan')))
                return

            depth = depth_image[y, x]
            if depth == 0:
                rospy.logwarn("Cannot find QR code (depth = 0)")
                self.distance_pub.publish(Float32(data=float('nan')))
                return

            # Find QR code, detect its distance and publish it
            distance = depth * 0.001  # mm → m
            rospy.loginfo(f"Distance forward QR code: {distance:.2f} m")
            self.distance_pub.publish(Float32(data=distance))
        except Exception as e:
            rospy.logerr(f"Depth Process Error: {e}")
            self.distance_pub.publish(Float32(data=float('nan')))

if __name__ == "__main__":
    try:
        QRDepthReader()
    except rospy.ROSInterruptException:
        pass
