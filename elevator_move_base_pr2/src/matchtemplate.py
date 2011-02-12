#!/usr/bin/env python

import roslib; roslib.load_manifest('elevator_move_base_pr2')
import rospy
from sensor_msgs.msg import Image
#from std_msgs.msg import String
from elevator_move_base_pr2.msg import StringStamped
import cv
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
latest_msg = None
image_sub = None
debug_pub = None

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

    reslist = []
    for template in templates:
        temptype = template[0]
        tempimg  = template[1]
        tempthre = template[2]
        tempname = template[3]
        if template[4] == 'CCORR': tempmethod = cv.CV_TM_CCORR_NORMED
        elif template[4] == 'CCOEFF': tempmethod = cv.CV_TM_CCOEFF_NORMED
        else: tempmethod = cv.CV_TM_SQDIFF_NORMED
        ressize = list(cv.GetSize(cv_image))
        ressize[0] -= cv.GetSize(tempimg)[0] - 1
        ressize[1] -= cv.GetSize(tempimg)[1] - 1
        results = cv.CreateImage(ressize, cv.IPL_DEPTH_32F, 1 )
        cv.MatchTemplate(cv_image, tempimg, results, cv.CV_TM_SQDIFF_NORMED)

        status = cv.MinMaxLoc(results)

        if (tempmethod == cv.CV_TM_SQDIFF_NORMED and status[0] < tempthre) or (tempmethod != cv.CV_TM_SQDIFF_NORMED and tempthre < status[1]):
            result.data += tempname+' '
            reslist += [(tempname,status)]

    result_pub.publish(result)
    publish_debug(cv_image, reslist)

def publish_debug(img, results):
    imgsize = cv.GetSize(img)
    sizelist = [cv.GetSize(tmp[1]) for tmp in templates]
    width = max(imgsize[0], sum([s[0] for s in sizelist]))
    height = imgsize[1] + max([s[1] for s in sizelist])
    output = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
    cv.Zero(output)
    cur_x = 0

    view_rect = (0, height-imgsize[1], imgsize[0], imgsize[1])
    cv.SetImageROI(output, view_rect)
    cv.Copy(img, output)
    for template in templates:
        size = cv.GetSize(template[1])
        cv.SetImageROI(output, (cur_x, 0, size[0], size[1]))
        cv.Copy(template[1], output)
        cur_x += size[0]
        for _,status in [s for s in results if s[0] == template[3]]:
            print status
            cv.Rectangle(output, (0, 0), size, cv.RGB(255,255,255), 9)
        cv.SetImageROI(output, view_rect)
        for _,status in [s for s in results if s[0] == template[3]]:
            pt2 = (status[2][0]+size[0], status[2][1]+size[1])
            cv.Rectangle(output, status[2], pt2, cv.RGB(255,255,255), 5)

    cv.ResetImageROI(output)
    debug_pub.publish(bridge.cv_to_imgmsg(output, encoding="passthrough"))

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
    templates = [[typename,cv.LoadImage(rospy.get_param('~template/'+typename+'/path'),cv.CV_LOAD_IMAGE_GRAYSCALE),rospy.get_param('~template/'+typename+'/thre'),rospy.get_param('~template/'+typename+'/name',''),rospy.get_param('~template/'+typename+'/method','')] for typename in template_list]

    result_pub = rospy.Publisher("~result",StringStamped,MySubscribeListener())
    debug_pub = rospy.Publisher("~debug_image",Image)

    while not rospy.is_shutdown():
        process_msg()
        rospy.sleep(0.1)

    result_pub.unregister()
    debug_pub.unregister()
    if image_sub != None:
        image_sub.unregister()

