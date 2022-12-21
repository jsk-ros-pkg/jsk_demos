#!/usr/bin/env python
import rospy
import message_filters
from jsk_recognition_msgs.msg import ClassificationResult,RectArray
rospy.init_node("find_ball")


def callback_each(class_msg,rect_msg,label_name,publisher):
    # print "callback"
    # print(class_msg.labels)
    # print(class_msg.label_names)
    ball_rect_list = []
    if len(rect_msg.rects)>=len(class_msg.label_names):
        for i,v in enumerate(class_msg.label_names):
            if v==label_name :
                ball_rect_list.append(rect_msg.rects[i])
    if len(ball_rect_list) > 0:
        ball_rectarray = RectArray()
        ball_rectarray.header = rect_msg.header
        ball_rectarray.rects = ball_rect_list
        print(ball_rectarray)
        print(label_name,class_msg.label_proba[0])
        publisher.publish(ball_rectarray)

def callback(class_msg,rect_msg):
    callback_each(class_msg,rect_msg,"lined_ball",pub_lined_ball_rect)
    callback_each(class_msg,rect_msg,"yellow_ball",pub_yellow_ball_rect)
    callback_each(class_msg,rect_msg,"white_ball",pub_white_ball_rect)

pub_lined_ball_rect = rospy.Publisher('/find_ball/lined_ball_rect',RectArray , queue_size=1)
pub_yellow_ball_rect = rospy.Publisher('/find_ball/yellow_ball_rect',RectArray , queue_size=1)
pub_white_ball_rect = rospy.Publisher('/find_ball/white_ball_rect',RectArray , queue_size=1)


sub_class = message_filters.Subscriber('/edgetpu_object_detector/output/class', ClassificationResult)
sub_rect = message_filters.Subscriber('/edgetpu_object_detector/output/rects', RectArray)

sync = message_filters.ApproximateTimeSynchronizer([sub_class,sub_rect], 10, 0.5)
sync.registerCallback(callback)

print "start"
rospy.spin()
