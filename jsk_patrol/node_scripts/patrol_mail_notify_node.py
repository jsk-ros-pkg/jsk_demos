#!/usr/bin/env python
# coding: utf-8

from collections import Counter
import datetime
import os
import shlex
import shutil
import subprocess
import tempfile

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from jsk_recognition_msgs.msg import ClassificationResult
import message_filters
import cv2

from jsk_patrol.srv import PatrolMailNotify
from jsk_patrol.srv import PatrolMailNotifyResponse


def get_greeting():
    current_time = datetime.datetime.now()

    if current_time.hour < 4:
        return "こんばんは。\n\n"
    elif current_time.hour < 10:
        return "おはよう。\n\n"
    elif current_time.hour < 17:
        return "こんにちは。\n\n"
    else:
        return "こんばんは。\n\n"


class MailNotify(object):

    def __init__(self):
        super(MailNotify, self).__init__()
        self.labelindex2names = rospy.get_param('~labelindex2names', None)
        self.bridge = cv_bridge.CvBridge()
        self.wait_n_topic = rospy.get_param('~wait_n_topic', 1)
        if self.wait_n_topic < 0:
            raise ValueError("wait n topic should be greater than 0")
        self.tweet_pub = rospy.Publisher('~tweet', String)
        self.service = rospy.Service(
            '~notify',
            PatrolMailNotify, self.request_callback)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 100)
        sub_img = message_filters.Subscriber(
            "~input/image",
            Image, queue_size=100, buff_size=2**24)
        sub_class = message_filters.Subscriber(
            '~input/class',
            ClassificationResult, queue_size=100, buff_size=2**24)
        self.subs = [sub_img, sub_class]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, img_msg, class_result_msg):
        self.img_msg.append(img_msg)
        self.class_result_msg.append(class_result_msg)

    def request_callback(self, req):
        place_name = req.place_name.data
        mail_title = req.mail_title.data
        from_address = req.from_address.data
        to_address = req.to_address.data
        message = req.message.data

        self.img_msg = []
        self.class_result_msg = []

        self.subscribe()

        start_time = rospy.Time.now()
        r = rospy.Rate(1)

        while not rospy.is_shutdown() and \
              len(self.img_msg) <= self.wait_n_topic:
            if rospy.Time.now() - start_time > rospy.Duration(30):
                break
            r.sleep()

        self.unsubscribe()

        if len(self.img_msg) == 0:
            return PatrolMailNotifyResponse()
        self.img_msg = self.img_msg[-1]
        self.class_result_msg = self.class_result_msg[-1]

        bridge = self.bridge
        img = bridge.imgmsg_to_cv2(self.img_msg, desired_encoding='bgr8')

        # create temp directory
        dirname = tempfile.mkdtemp()
        os.chmod(dirname, 0777)

        img_path = os.path.join(dirname, 'detected-img.jpg')
        cv2.imwrite(img_path, img)
        os.chmod(img_path, 0777)
        greeting_text = get_greeting()
        if len(self.class_result_msg.labels) == 0:
            body_text = "{}今日の{}は綺麗だね。".\
                format(greeting_text, place_name)
        else:
            body_text = '{}{}にものがあるのを見つけたよ。\n見つかったものは...\n'.\
                format(greeting_text, place_name)
            c = Counter(self.class_result_msg.labels)
            for index, num in c.items():
                if self.labelindex2names is None:
                    body_text += ' {} {}個\n'.format(
                        self.class_result_msg.target_names[index], num)
                else:
                    body_text += ' {} {}個\n'.format(
                        self.labelindex2names[index], num)
        if len(message) > 0:
            body_text += "\n\n{}".format(message)
        text_cmd = 'echo -e "' + body_text + '"'
        process1 = subprocess.Popen(shlex.split(text_cmd),
                                    stdout=subprocess.PIPE)
        mail_cmd = 'mail -s "{}" -a {} -r {} {}'.format(
            mail_title,
            img_path,
            from_address,
            to_address)
        subprocess.Popen(shlex.split(mail_cmd),
                         stdin=process1.stdout)

        # tweet message
        tweet_message = body_text + img_path
        self.tweet_pub.publish(String(data=tweet_message))

        rospy.sleep(4.0)

        # delete temp directory
        shutil.rmtree(dirname)

        return PatrolMailNotifyResponse()


if __name__ == '__main__':
    # for japanese settings
    import sys
    reload(sys)
    sys.setdefaultencoding('utf-8')

    # init node
    rospy.init_node('mail_notify')
    node = MailNotify()
    rospy.spin()
