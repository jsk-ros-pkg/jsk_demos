#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
from roseus.msg import StringStamped
import cv
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
latest_msg = None
image_sub = None
debug_pub = None

# for cv_bridge back compatibility
def encoding_as_cvtype(encoding):
    from cv_bridge.boost.cv_bridge_boost import getCvType
    try:
        return getCvType(encoding)
    except RuntimeError as e:
        raise CvBridgeError(e)


def imgmsg_to_cv(img_msg, desired_encoding = "passthrough"):
    try:
        return bridge.imgmsg_to_cv(img_msg, desired_encoding)
    except:
        cv2_im = bridge.imgmsg_to_cv2(img_msg, desired_encoding)
        img_msg = bridge.cv2_to_imgmsg(cv2_im)
        source_type = encoding_as_cvtype(img_msg.encoding)
        im = cv.CreateMatHeader(img_msg.height, img_msg.width, source_type)
        cv.SetData(im, img_msg.data, img_msg.step)
        return im


def cv_to_imgmsg(cvim, encoding = "passthrough"):
    try:
        return bridge.cv_to_imgmsg(cvim, encoding)
    except:
        img_msg = Image()
        (img_msg.width, img_msg.height) = cv.GetSize(cvim)
        if encoding == "passthrough":
            img_msg.encoding = bridge.cvtype_to_name[cv.GetElemType(cvim)]
        else:
            img_msg.encoding = encoding
            if encoding_as_cvtype(encoding) != cv.GetElemType(cvim):
                raise CvBridgeError, "invalid encoding"
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) / img_msg.height
        return img_msg


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
        cv_image = imgmsg_to_cv(msg, "mono8")
    except CvBridgeError, e:
        print(e)

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
        cv.MatchTemplate(cv_image, tempimg, results, tempmethod)

        status = cv.MinMaxLoc(results)
        if (tempmethod == cv.CV_TM_SQDIFF_NORMED):
            found = ( status[0] < tempthre ) 
            reslist += [(tempname,(status[0],status[2],tempthre,found))]
            if found :
                result.data += tempname+' '
        else :
            found = (tempthre < status[1] )
            reslist += [(tempname,(status[1],status[3],tempthre,found))]
            if found :
                result.data += tempname+' '
        print(reslist)

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

        #cv.PutText(output, tempname, (0,size[1]-16), font, cv.CV_RGB(255,255,255))
        #cv.PutText(output, str(tempthre)+'<'+str(status[1]), (0,size[1]-8), font, cv.CV_RGB(255,255,255))
        for _,status in [s for s in results if s[0] == template[3]]:
            print(status)
            cv.PutText(output, template[3], (0,size[1]-42), font, cv.CV_RGB(255,255,255))
            cv.PutText(output, "%7.5f"%(status[0]), (0,size[1]-24), font, cv.CV_RGB(255,255,255))
            cv.PutText(output, "%7.5f"%(status[2]), (0,size[1]-8), font, cv.CV_RGB(255,255,255))
            if status[3] : 
                cv.Rectangle(output, (0, 0), size, cv.RGB(255,255,255), 9)
        cv.SetImageROI(output, view_rect)
        for _,status in [s for s in results if s[0] == template[3]]:
            pt2 = (status[1][0]+size[0], status[1][1]+size[1])
            if status[3] : 
                cv.Rectangle(output, status[1], pt2, cv.RGB(255,255,255), 5)

    cv.ResetImageROI(output)
    debug_pub.publish(cv_to_imgmsg(output, encoding="passthrough"))

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

def resolve_ros_path(path):
    if "package://" in path:
        plist = path.split('/')
        pkg_dir = roslib.packages.get_pkg_dir(plist[2])
        return "/".join([pkg_dir] + plist[3:])
    return path

if __name__=='__main__':
    rospy.init_node('match_template')

    global templates, result_pub, font
    font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0.0, 1, cv.CV_AA)
    template_list = rospy.get_param('~template_list').split()
    templates = [[typename,cv.LoadImage(resolve_ros_path(rospy.get_param('~template/'+typename+'/path')),cv.CV_LOAD_IMAGE_GRAYSCALE),rospy.get_param('~template/'+typename+'/thre'),rospy.get_param('~template/'+typename+'/name',''),rospy.get_param('~template/'+typename+'/method','')] for typename in template_list]

    result_pub = rospy.Publisher("~result", StringStamped, MySubscribeListener(), queue_size=1)
    debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)

    try:
        while not rospy.is_shutdown():
            process_msg()
            rospy.sleep(0.1)
    except rospy.ROSInterruptException: pass

    result_pub.unregister()
    debug_pub.unregister()
    if image_sub != None:
        image_sub.unregister()

