#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import namedtuple
from urlparse import urlparse

import cv2
import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import matplotlib
matplotlib.use('Agg')  # NOQA
import matplotlib.pyplot as plt
import numpy as np
from roseus.msg import StringStamped
import rospkg
import rospy
from sensor_msgs.msg import Image


Template = namedtuple('Template', ['name', 'image', 'thre', 'method'])
Result = namedtuple('Result', ['score', 'found', 'top_left'])


class MatchTemplate(ConnectionBasedTransport):
    def __init__(self):
        super(MatchTemplate, self).__init__()
        self.rospack = rospkg.RosPack()
        self.cv_bridge = cv_bridge.CvBridge()
        self.pub_result = self.advertise(
            '~result', StringStamped, queue_size=1)
        self.pub_debug = self.advertise(
            '~debug_image', Image, queue_size=1)
        self.templates = self.load_templates()
        rospy.loginfo('Initialized with %d templates' % len(self.templates))

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~image', Image, self.image_cb, queue_size=1)

    def unsubscribe(self):
        self.sub.unregister()

    def load_templates(self):
        method = rospy.get_param('~method', '')
        if method == 'CCORR':
            method = cv2.TM_CCORR_NORMED
        elif method == 'CCOEFF':
            method = cv2.TM_CCOEFF_NORMED
        elif method == 'SQDIFF':
            method = cv2.TM_SQDIFF_NORMED
        else:
            method = cv2.TM_SQDIFF_NORMED

        template_list = rospy.get_param('~template_list').split()
        templates = {}
        for typename in template_list:
            prefix = '~template/' + typename
            imgpath = self.resolve_ros_path(rospy.get_param(prefix + '/path'))

            templates[typename] = Template(
                name=rospy.get_param(prefix + '/name'),
                thre=rospy.get_param(prefix + '/thre'),
                method=method,
                image=cv2.imread(imgpath, cv2.IMREAD_GRAYSCALE),
            )
        return templates

    def resolve_ros_path(self, path):
        path = path.strip()
        if path.startswith('package://'):
            uri = urlparse(path)
            pkgdir = self.rospack.get_path(uri.hostname)
            return pkgdir + uri.path
        else:
            return path

    def image_cb(self, msg):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return

        results = dict()
        for typename, template in sorted(self.templates.items()):
            res = cv2.matchTemplate(img, template.image, template.method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            if template.method == cv2.TM_SQDIFF_NORMED:
                score = min_val
                result = Result(
                    score=score,
                    found=min_val < template.thre,
                    top_left=min_loc)
            else:
                score = max_val
                result = Result(
                    score=score,
                    found=max_val > template.thre,
                    top_left=max_loc)
            results[template.name] = result

            rospy.loginfo('score of %s: %f' % (template.name, score))

        # publish result
        msg = StringStamped(header=msg.header)
        msg.data = ' '.join([n for n, r in results.items() if r.found])
        self.pub_result.publish(msg)

        # publish debug image
        self.publish_debug(img, results)

    def publish_debug(self, img, results):
        templates = self.templates.values()
        templates.sort(key=lambda t: t.name)
        imgs = [t.image for t in templates]

        tmpl_img = np.hstack(imgs)
        scale = 1.0 * tmpl_img.shape[1] / img.shape[1]
        if scale >= 1.0:
            img_scale = scale
            tmpl_scale = 1.0
            img = cv2.resize(img, None,
                             fx=img_scale, fy=img_scale)
        else:
            tmpl_scale = 1.0 / scale
            img_scale = 1.0
            tmpl_img = cv2.resize(tmpl_img, None,
                                  fx=tmpl_scale, fy=tmpl_scale)

        debug = np.vstack((tmpl_img, img))
        debug = cv2.cvtColor(debug, cv2.COLOR_GRAY2BGR)

        fig = plt.figure(dpi=200)
        ax = fig.add_subplot(1, 1, 1)
        ax.set_axis_off()
        ax.imshow(debug)

        # draw roi if found
        for i, t in enumerate(templates):
            res = results[t.name]
            w, h = t.image.shape[1] * tmpl_scale, t.image.shape[0] * tmpl_scale
            ax.text(i * w + 20, h + 50,
                    '%s:\n%.2f' % (t.name, res.score),
                    fontsize=8,
                    bbox={'facecolor': 'white',
                          'alpha': 0.5,
                          'pad': 3})

            if res.found:
                ax.add_patch(plt.Rectangle(
                    (i * w, 0), w, h,
                    fill=False,
                    edgecolor=(1.0, 0.0, 0.0),
                    linewidth=2))

                top_left = (res.top_left[0] * img_scale,
                            res.top_left[1] * img_scale + tmpl_img.shape[0])
                w = t.image.shape[1] * img_scale
                h = t.image.shape[0] * img_scale
                ax.add_patch(plt.Rectangle(
                    top_left, w, h,
                    fill=False,
                    edgecolor=(1.0, 0.0, 0.0),
                    linewidth=2))

        plt.margins(0, 0)
        plt.subplots_adjust(
            left=0, right=1, top=1, bottom=0, hspace=0, wspace=0)
        fig = plt.gcf()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        debug = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
        debug.shape = (h, w, 3)
        fig.clf()
        plt.close()

        try:
            msg = self.cv_bridge.cv2_to_imgmsg(debug, 'rgb8')
            self.pub_debug.publish(msg)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('match_template')
    n = MatchTemplate()
    rospy.spin()
