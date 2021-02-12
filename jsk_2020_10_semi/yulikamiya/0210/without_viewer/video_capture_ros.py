import cv2
import datetime

import numpy as np
import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import CompressedImage

class VideoCapture(ConnectionBasedTransport):

    def __init__(self, env, video_device):
        rospy.init_node('image_to_label')
        rospy.set_param('~always_subscribe', True)
        super(VideoCapture, self).__init__()
        self.cap = None
        print("new video coming")
        try:
            self.CROPPED_IMAGE_WIDTH = env['CroppedImageWidth']
            self.CROPPED_IMAGE_HEIGHT = env['CroppedImageHeight']
        except KeyError:
            print('Invalid config file')
            raise

        print('video_device:', video_device)
        #self.cap = cv2.VideoCapture(video_device)
        #if not self.cap.isOpened():
        #    raise RuntimeError('Cannot open camera')
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

    def read(self):
        self._sub = rospy.Subscriber('/head_camera/rgb/image_raw/compressed', CompressedImage, self.image_cb, queue_size=1, buff_size=2**26)
        # clear buffer of VideoCapture
        if (self.cap):
            ("subscribe success")
            for i in range(5):
                self.cap.grab()

            ret, frame = self.cap.read()
            if ret is False or frame is None:
                return None
            # crop
            cx = frame.shape[1] / 2
            cy = frame.shape[0] / 2
            frame_ = frame[int(cy-self.CROPPED_IMAGE_HEIGHT/2):int(cy+self.CROPPED_IMAGE_HEIGHT/2),
                        int(cx-self.CROPPED_IMAGE_WIDTH/2):int(cx+self.CROPPED_IMAGE_WIDTH/2)]
            return frame_

    def image_cb(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.cap = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.cap = self.cap[:, :, ::-1]
        # do job
        cv2.imshow("image", self.cap)
        cv2.waitKey(1)

    def release(self):
        if self.cap is not None:
            self.cap.release()
