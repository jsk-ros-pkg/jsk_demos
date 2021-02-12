#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from std_msgs.msg import String
import cv_bridge
from geometry_msgs.msg import Vector3Stamped
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import CompressedImage

class TemperatureRecognition():
    def __init__(self):
        rospy.set_param('~always_subscribe', True)
        super(TemperatureRecognition, self).__init__()
        pub_go_back = rospy.Publisher('go_back', String, queue_size=10)
        pub_matched_face = rospy.Publisher('matched_face', Vector3Stamped, queue_size=10)
        self.subscribe_name()
        self.subscribe_image()
        self._color = "gray"
    
    def subscribe_name(self):
        self._sub_name = rospy.Subscriber("matched_face", Vector3Stamped, self.name_cb)

    def subscribe_image(self):
        self._sub_image = rospy.Subscriber('head_camera/rgb/image_raw/compressed',CompressedImage, self.image_cb, queue_size=1, buff_size=2**26)

    def image_cb(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = img[390:470, 390:450, ::-1]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        colors = np.mean(np.mean(hsv, axis=1), axis=0)
        # green [50-60, 200-220, 140-170]
        # yellow [20-35, ??, ??]
        # red [120-130, 240-255, 170-180]
        if (colors[2]<100 or colors[1]<150):
            self._color = "gray"
        elif(colors[0]>20 and colors[0]<35):
            self._color = "yellow"
        elif(colors[0]>50 and colors[0]<60):
            self._color = "green"
        elif(colors[0]>120 and colors[0]<130):
            self._color = "red"
        print(self._color)
        cv2.imshow("image", hsv)
        cv2.waitKey(1)

    def name_cb(self, msg):
        print("name called")
        if(self._color = "green" or self._color = "yellow" or (self._color = red and count = 3)):
            new_msg = "go"
            pub_go_back.publish(new_msg)
        elif(self._color = "red"):
            new_msg = Vector3Stamped()
            new_msg.header.frame_id = msg.header.frame_id
            new_msg.vector.x = msg.vector.x
            new_msg.vector.y = msg.vector.y
            new_msg.vector.z = msg.vector.z+1
            pub_matched_face.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('temperature_recognition', anonymous = True)
    try:
        temperature_recognition = TemperatureRecognition()
    except rospy.ROSInterruptException: pass
    rospy.spin()