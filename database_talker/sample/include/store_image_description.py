#!/usr/bin/env python

import argparse
import sys

import rospy
import actionlib
import std_msgs.msg
from sensor_msgs.msg import CompressedImage
import os
from importlib import import_module

import cv2
IsHeadless = False
if 'DISPLAY' not in os.environ:
    IsHeadless = True
import numpy as np
from cv_bridge import CvBridge

import json
import base64
from openai_ros.srv import ChatCompletions, ChatCompletionsRequest

from std_msgs.msg import Header
from jsk_recognition_msgs.msg import VQATaskActionResult, VQATaskResult, VQAResult, QuestionAndAnswerText

from roseus.msg import StringStamped

bridge = CvBridge()
result_pub = None
image_pub = None
chat_completion = None
images = [np.array(cv2.imencode('.jpg',  np.zeros((120,160,3), np.uint8))[1]).tostring()]
answers = []

vqa_model = 'gpt-4o-mini'
chat_model = 'gpt-4o-mini'
vqa_temperature = 0.5
chat_temperature = 0.3

def vqa(question, images, temperature = 0.0, max_tokens = 300, debug = False):
    global chat_completion

    # debug
    if (not IsHeadless) and debug and len(images)>0:
        cv2.imshow('debug', cv2.hconcat([cv2.imdecode(np.fromstring(image, np.uint8), cv2.IMREAD_COLOR) for image in images]))
        cv2.waitKey(100)

    image_urls = [{'type': 'image_url', 'image_url' : {'url': 'data:image/jpeg;base64,'+base64.b64encode(image).decode('utf-8')}} for image in images]
    req = ChatCompletionsRequest(model = vqa_model,
                                 messages = json.dumps([{"role": "user",
                                                         "content": [ {"type": "text", "text": question} ]
                                                         + image_urls }]),
                                 temperature = temperature, max_tokens=max_tokens)
    rospy.loginfo("{}".format(req.messages[0:255]))

    ret = chat_completion(req)
    answer = ret.content

    rospy.loginfo('Q: {}'.format(question))
    rospy.loginfo('- {}'.format(answer))

    return answer

debug_msg = StringStamped()
def debug_cb(data):
    connection_header =  data._connection_header['type'].split('/')
    ros_pkg = connection_header[0] + '.msg'
    msg_type = connection_header[1]
    msg_class = getattr(import_module(ros_pkg), msg_type)
    rospy.loginfo("received {}/{}".format(ros_pkg, msg_type))
    global debug_msg
    if msg_class is StringStamped:
        msg = msg_class().deserialize(data._buff)
        debug_msg = msg
    else:
        debug_msg = StringStamped(header = rospy.Header(stamp=rospy.Time.now()), data="debug")
    #
    return
    global mongodb_event_sub
    mongodb_event_sub.unregister()
    mongodb_event_sub = rospy.Subscriber('/publish_trigger_mongodb_event', rospy.AnyMsg, debug_cb, queue_size=1)
    
def cb(msg):
    global chat_completion
    global images, answers
    global result_pub, image_pub
    global debug_msg

    small_msg_data = np.array(cv2.imencode('.jpg', cv2.resize(cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR),(160,120)))[1]).tostring()
    if len(images) == 0:
        images.extend([small_msg_data])

    questions = rospy.get_param('~questions', ['Provide a brief caption under 140 characters for this image, focusing on the most striking aspect and overall atmosphere. If the images is black, blurred, disturbed or shows meaningless objects, answer "NO"'])
    question = ' '.join(questions) if type(questions) == list else questions

    if (not IsHeadless):
        cv2.imshow('debug', cv2.hconcat([cv2.imdecode(np.fromstring(image, np.uint8), cv2.IMREAD_COLOR) for image in images]))
        cv2.waitKey(100)

    # use VQA to filter new image (DO NOT USE THIS, THIS COSTS TOO HIGH)
    '''
    use_this_image_answer = vqa(# "Does the last image is totally different from and more impressive than the rest of images?, Please answer YES or NO.",
                                # "Focusing on the subject matter of the images, is the last image portraying a completely different theme or subject than the earlier images? Please respond with YES or NO and identify the theme or subject of all images.",
                                "Focusing on the subject matter of the images, is the first image portraying a completely different theme or subject than the {}? Please respond with YES or NO and identify the theme or subject of all images.".format('earlier images' if len(images)>1 else 'other image'),
                                [small_msg_data] + images, temperature = 1.0, debug=True)
    use_this_image = 'YES' in use_this_image_answer[:10]
    '''
    elapsed_from_trigger = abs((rospy.Time.now() - debug_msg.header.stamp).to_sec())
    rospy.loginfo("received images, {} sec after trigger event".format(elapsed_from_trigger))
    if elapsed_from_trigger < 5 and debug_msg.data == 'debug':
        images.extend([small_msg_data])
        if len(images) > 10:
            images = images[1:]

        try:
            answer = vqa(question, [msg.data], temperature = vqa_temperature)
            if answer == 'NO':
                raise Exception('Invalid image')
            rospy.loginfo("- {}".format(answer))
            for a in answers:
                rospy.loginfo(" .. {}".format(a))
            req = ChatCompletionsRequest(model = chat_model,
                                         messages = json.dumps([{"role": "system", "content": "You can compare whether your sentenses describe the same scene and returns with 'YES' or 'NO'"},
                                                                {"role": "user", "content": "Return 'YES' if given text '{}' is similar to one of the following list '{}', otherwise return 'NO'".format(answer, answers)}
                                         ]), temperature = chat_temperature)
            rospy.loginfo("Q: {}".format(req.messages[0:255]))
            ret = chat_completion(req)
            rospy.loginfo("A: {}".format(ret.content))
            if ret.content == 'YES':
                raise Exception('Duplicates image')
            answers.extend([answer])
            if len(answers) > 5:
                answers = answers[1:]
            result_pub.publish(VQATaskActionResult(header=Header(stamp=rospy.Time.now()),
                                                   result=VQATaskResult(result=VQAResult(result=[QuestionAndAnswerText(question=question, answer=answer)]), done=True)))
            image_pub.publish(msg)
        except Exception as e:
            filename = '/tmp/image.jpg'
            rospy.logerr("write current image to {}, due to {}".format(filename, e))
            cv2.imwrite(filename, cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR))
    return

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--file', type=str, dest='filename')
        args = parser.parse_args(rospy.myargv()[1:])

        rospy.init_node('store_image_description', anonymous=True)
        debug_msg = StringStamped(header=Header(stamp=rospy.Time.now()))
        rospy.loginfo("wait for '/openai/chat_completions'")
        rospy.wait_for_service('/openai/chat_completions')
        chat_completion = rospy.ServiceProxy('/openai/chat_completions', ChatCompletions)

        result_pub = rospy.Publisher("~result", VQATaskActionResult, queue_size=1)
        image_pub = rospy.Publisher("~result/image/compressed", CompressedImage, queue_size=1)
        mongodb_event_sub = rospy.Subscriber('/publish_trigger_mongodb_event', rospy.AnyMsg, debug_cb, queue_size=1)

        # debug mode
        print(args.filename)
        if args.filename:
            try:
                image = cv2.imread(args.filename)
                ret, buffer = cv2.imencode('.jpg', image)
                msg = CompressedImage()
                msg.header = std_msgs.msg.Header()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(buffer).tobytes()
                debug_msg.data = 'debug'
                cb(msg)
            except Exception as e:
                rospy.logerr('could not find image file ({})'.format(e))
            sys.exit(0)

        rospy.Subscriber('image', CompressedImage, cb, queue_size=1)
        rospy.loginfo("start subscribing {}".format(rospy.resolve_name('image')))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
