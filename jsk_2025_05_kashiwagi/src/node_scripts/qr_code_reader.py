#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode

class QRReader:
    def __init__(self):
        rospy.init_node('qr_reader')
        self.current_kashiwagi_state = "unknown"
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.state_sub = rospy.Subscriber('/kashiwagi_state', String, self.state_callback)
        self.text_pub = rospy.Publisher('/qr_data', String, queue_size=10)
        self.pos_pub = rospy.Publisher('/qr_position', Point, queue_size=10)
        rospy.loginfo("Launching QR code reader node ....")
        rospy.spin()

    def state_callback(self, msg):
        self.current_kashiwagi_state = msg.data

    def image_callback(self, msg):
        if self.current_kashiwagi_state != "talking_game:listening_turn":
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        qr_codes = decode(frame)

        for obj in qr_codes:
            data = obj.data.decode('utf-8')
            rospy.loginfo(f"QR data : {data}")
            self.text_pub.publish(data)

            pts = obj.polygon
            if len(pts) == 4:
                cx = sum([p.x for p in pts]) / 4
                cy = sum([p.y for p in pts]) / 4
                rospy.loginfo(f"QR position: x={cx:.1f}, y={cy:.1f}")

                point = Point()
                point.x = cx
                point.y = cy
                point.z = 0
                self.pos_pub.publish(point)

if __name__ == '__main__':
    try:
        QRReader()
    except rospy.ROSInterruptException:
        pass
