#!/usr/bin/env python

import roslib; roslib.load_manifest('elevator_move_base_pr2')
import rospy
from sensor_msgs.msg import Image
#from std_msgs.msg import String
from elevator_move_base_pr2.msg import StringStamped
from cv import *
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
latest_msg = None
image_sub = None

def image_callback (msg):
    global latest_msg
    latest_msg = msg
    return

def process_msg ():
    global latest_msg
    msg = latest_msg
    latest_msg = None
    if msg == None: return

    result = StringStamped(data='',header=msg.header)

    try:
        cv_image = bridge.imgmsg_to_cv(msg, "mono8")
    except CvBridgeError, e:
        print e

    for template in templates:
        temptype = template[0]
        tempimg  = template[1]
        tempthre = template[2]
        tempname = template[3]
        if template[4] == 'CCORR': tempmethod = CV_TM_CCORR_NORMED
        elif template[4] == 'CCOEFF': tempmethod = CV_TM_CCOEFF_NORMED
        else: tempmethod = CV_TM_SQDIFF_NORMED
        ressize = list(GetSize(cv_image))
        ressize[0] -= GetSize(tempimg)[0] - 1
        ressize[1] -= GetSize(tempimg)[1] - 1
        results = CreateImage(ressize, IPL_DEPTH_32F, 1 )
        MatchTemplate(cv_image, tempimg, results, CV_TM_SQDIFF_NORMED)

        status = MinMaxLoc(results)

        if (tempmethod == CV_TM_SQDIFF_NORMED and status[0] < tempthre) or (tempmethod != CV_TM_SQDIFF_NORMED and tempthre < status[1]):
            result.data += tempname+' '

    result_pub.publish(result)

class MySubscribeListener(rospy.SubscribeListener):
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        global image_sub
        if image_sub is None:
            image_sub = rospy.Subscriber(rospy.resolve_name("~image"),Image,image_callback,queue_size=1)
    def peer_unsubscribe(self, topic_name, num_peers):
        global image_sub
        if num_peers == 0:
            image_sub.unregister()
            image_sub = None

if __name__=='__main__':
    rospy.init_node('match_template')

    global templates, result_pub
    template_list = rospy.get_param('~template_list').split()
    templates = [[typename,LoadImage(rospy.get_param('~template/'+typename+'/path'),CV_LOAD_IMAGE_GRAYSCALE),rospy.get_param('~template/'+typename+'/thre'),rospy.get_param('~template/'+typename+'/name',''),rospy.get_param('~template/'+typename+'/method','')] for typename in template_list]

    result_pub = rospy.Publisher("~result",StringStamped,MySubscribeListener())

    while not rospy.is_shutdown():
        process_msg()
        rospy.sleep(0.1)

    result_pub.unregister()
    if image_sub != None:
        image_sub.unregister()

