#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

import actionlib
from bson import json_util
import cv2
import datetime
import difflib
import json
import random
import rospkg
import tempfile
import time
import traceback


from cv_bridge import CvBridge


from googletrans import Translator
from googletrans.models import Translated


from mongodb_store.util import deserialise_message

from google_chat_ros.msg import Card, Section, WidgetMarkup, Image
from google_chat_ros.msg import SendMessageAction, SendMessageGoal

from mongodb_store_msgs.msg import StringPairList, StringPair
from mongodb_store_msgs.srv import MongoQueryMsg, MongoQueryMsgRequest

from ros_google_cloud_language.msg import AnalyzeTextAction
from ros_google_cloud_language.msg import AnalyzeTextGoal

from dialogflow_task_executive.msg import DialogTextAction
from dialogflow_task_executive.msg import DialogTextGoal
from dialogflow_task_executive.msg import DialogTextActionResult

from jsk_recognition_msgs.msg import ClassificationTaskAction
from jsk_recognition_msgs.msg import ClassificationTaskGoal
from jsk_recognition_msgs.msg import VQATaskAction
from jsk_recognition_msgs.msg import VQATaskGoal

from openai_ros.srv import Completion

from dateutil import tz

bridge = CvBridge()
translator = Translator()
JST = tz.gettz('Asia/Tokyo')


class MessageListener(object):

    def __init__(self):
        rospy.loginfo("wait for '/google_chat_ros/send'")
        self.chat_ros_ac = actionlib.SimpleActionClient(
            '/google_chat_ros/send', SendMessageAction)
        self.chat_ros_ac.wait_for_server()
        # self.pub = rospy.Publisher(
        #    '/google_chat_ros/send/goal', SendMessageActionGoal, queue_size=1)

        rospy.loginfo("wait for '/message_store/query_messages'")
        rospy.wait_for_service('/message_store/query_messages')
        self.query = rospy.ServiceProxy(
            '/message_store/query_messages', MongoQueryMsg)

        rospy.loginfo("wait for '/classification/inference_server'")
        self.classification_ac = actionlib.SimpleActionClient(
            '/classification/inference_server', ClassificationTaskAction)
        self.classification_ac.wait_for_server()

        rospy.loginfo("wait for '/vqa/inference_server'")
        self.vqa_ac = actionlib.SimpleActionClient(
            '/vqa/inference_server', VQATaskAction)
        self.vqa_ac.wait_for_server()

        # https://github.com/k-okada/openai_ros
        # this requres apt install python3.7 python3.7-venv
        rospy.loginfo("wait for '/openai/get_response'")
        rospy.wait_for_service('/openai/get_response')
        self.completion = rospy.ServiceProxy(
            '/openai/get_response', Completion)

        # integration of dialogflow <-> google_chat_ros was performed
        # by google_chat_ros/script/helper.py
        rospy.loginfo("wait for '/dialogflow_client/text_action'")
        self.dialogflow_ac = actionlib.SimpleActionClient(
            '/dialogflow_client/text_action', DialogTextAction)
        self.dialogflow_ac.wait_for_server()

        rospy.loginfo("wait for '/analyze_text/text'")
        self.analyze_text_ac = actionlib.SimpleActionClient(
            '/analyze_text/text', AnalyzeTextAction)
        self.analyze_text_ac.wait_for_server()

        # rospy.loginfo("subscribe '/google_chat_ros/message_activity'")
        # self.sub = rospy.Subscriber(
        #     '/google_chat_ros/message_activity', MessageEvent, self.cb)
        rospy.loginfo("subscribe '/dialogflow_client/text_action/result'")
        self.sub = rospy.Subscriber(
            '/dialogflow_client/text_action/result',
            DialogTextActionResult, self.cb
        )

        rospy.loginfo("all done, ready")

    def make_reply(self, message, lang="en"):
        rospy.logwarn("Run make_reply({})".format(message))
        query = self.text_to_salience(message)
        rospy.logwarn("query using salience word '{}'".format(query))
        # look for images
        try:
            # get chat message
            timestamp = datetime.datetime.now(JST)
            results, chat_msgs = self.query_dialogflow(
                query,
                timestamp,
                threshold=0.25
            )
            retry = 0
            while (retry < -1
                   and len(results) == 0
                   and len(chat_msgs.metas) > 0):
                meta = json.loads(chat_msgs.metas[-1].pairs[0].second)
                results, chat_msgs = self.query_dialogflow(
                    query,
                    datetime.datetime.fromtimestamp(
                        meta['timestamp']//1000000000, JST)
                )
                retry = retry + 1
            # sort based on similarity with 'query'
            chat_msgs_sorted = sorted(results,
                                      key=lambda x: x['similarity'],
                                      reverse=True)
            if len(chat_msgs_sorted) == 0:
                rospy.logwarn("no chat message was found")
            else:
                # query images that was taken when chat_msgs are stored
                # msg = chat_msgs_sorted[0]['msg']
                meta = chat_msgs_sorted[0]['meta']
                text = chat_msgs_sorted[0]['message']
                timestamp = chat_msgs_sorted[0]['timestamp']
                action = chat_msgs_sorted[0]['action']
                similarity = chat_msgs_sorted[0]['similarity']
                # query chat to get response
                # meta = json.loads(
                #     chat_msgs_sorted[0]['meta'].pairs[0].second)
                # text = msg.message.argument_text or msg.message.text
                # timestamp = datetime.datetime.fromtimestamp(
                #     meta['timestamp']//1000000000, JST)
                rospy.loginfo("Found message '{}'({}) at {}" +
                              "corresponds to query '{}' with {:2f}%".format(
                                  text,
                                  action,
                                  timestamp.strftime('%Y-%m-%d %H:%M:%S'),
                                  query,
                                  similarity
                              ))

            start_time = timestamp-datetime.timedelta(minutes=300)
            end_time = timestamp+datetime.timedelta(minutes=30)
            results = self.query_images_and_classify(query=query,
                                                     start_time=start_time,
                                                     end_time=end_time)

            end_time = results[-1]['timestamp']
            # sort
            results = sorted(results,
                             key=lambda x: x['similarities'],
                             reverse=True)
            rospy.loginfo("Probabilities of all images {}".format(
                list(map(lambda x: (x['label'], x['similarities']), results))))
            best_result = results[0]

            # if probability is too low, try again
            while len(results) > 0 and results[0]['similarities'] < 0.25:
                start_time = end_time-datetime.timedelta(hours=24)
                timestamp = datetime.datetime.now(JST)
                results = self.query_images_and_classify(
                    query=query, start_time=start_time,
                    end_time=end_time,
                    limit=300
                )
                if len(results) > 0:
                    end_time = results[-1]['timestamp']
                    # sort
                    results = sorted(
                        results, key=lambda x: x['similarities'], reverse=True)
                    # rospy.loginfo("Probabilities of all images {}".format(
                    #     list(map(lambda x: (x['label'], x['similarities']),
                    #              results))))
                    if len(results) > 0 and results[0]['similarities'] > \
                       best_result['similarities']:
                        best_result = results[0]

            rospy.loginfo(
                "Found '{}' image with {:0.2f} % simiarity at {}".format(
                    best_result['label'],
                    best_result['similarities'],
                    best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')
                ))

            # make prompt
            goal = VQATaskGoal()
            goal.compressed_image = best_result['image']

            # unusual objects
            if random.randint(0, 1) == 1:
                goal.questions = ['what unusual things can be seen?']
                reaction = 'and you saw '
            else:
                goal.questions = ['what is the atmosphere of this place?']
                reaction = 'and the atmosphere of the scene was '

            # get vqa result
            self.vqa_ac.send_goal(goal)
            self.vqa_ac.wait_for_result()
            result = self.vqa_ac.get_result()
            reaction += result.result.result[0].answer
            if (len(chat_msgs_sorted) > 0
                    and chat_msgs_sorted[0]['action']
                    and 'action' in chat_msgs_sorted[0]):
                reaction += " and you felt " + chat_msgs_sorted[0]['action']

            # make prompt
            prompt = ('if you are a pet and someone tells you \"' +
                      message +
                      '\" when we went together, ' +
                      reaction +
                      'in your memory of that moment, what would you reply? ' +
                      'Show only the reply in {lang}'.format(
                          lang={'en': 'English', 'ja': 'Japanese'}[lang]
                      ))
            result = self.completion(prompt=prompt, temperature=0)
            rospy.loginfo("prompt = {}".format(prompt))
            rospy.loginfo("result = {}".format(result))
            # pubish as card
            filename = tempfile.mktemp(
                suffix=".jpg", dir=rospkg.get_ros_home())
            self.write_image_with_annotation(filename, best_result, prompt)
            self.publish_google_chat_card(result.text, filename)

        except Exception as e:
            raise ValueError(
                "Query failed {} {}".format(e, traceback.format_exc()))

    def write_image_with_annotation(self, filename, best_result, prompt):
        image = bridge.compressed_imgmsg_to_cv2(best_result['image'])
        cv2.putText(
            image,
            "{} ({:.2f}) {}".format(
                best_result['label'],
                best_result['similarities'],
                best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')),
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 8, 1
        )
        cv2.putText(
            image,
            "{} ({:.2f}) {}".format(
                best_result['label'],
                best_result['similarities'],
                best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')),
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, 1
        )
        string_width = 70
        for i in range(0, len(prompt), string_width):
            # https://stackoverflow.com/questions/13673060/split-string-into-strings-by-length
            text = prompt[i:i+string_width]
            cv2.putText(image,
                        text,
                        (10, 43+int(i/string_width*20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 4, 1)
            cv2.putText(image,
                        text,
                        (10, 43+int(i/string_width*20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, 1)
        cv2.imwrite(filename, image)
        rospy.logwarn("save images to {}".format(filename))

    def query_dialogflow(self, query, end_time, limit=30, threshold=0.0):
        rospy.logwarn("Query dialogflow until {}".format(end_time))
        meta_query = {'inserted_at': {"$lt": end_time}}
        meta_tuple = (
            StringPair(MongoQueryMsgRequest.JSON_QUERY,
                       json.dumps(meta_query, default=json_util.default)),
        )
        chat_msgs = self.query(
            database='jsk_robot_lifelog',
            collection='fetch1075',
            # type='google_chat_ros/MessageEvent',
            type='dialogflow_task_executive/DialogTextActionResult',
            single=False,
            limit=limit,
            meta_query=StringPairList(meta_tuple),
            sort_query=StringPairList([StringPair('_meta.inserted_at', '-1')])
        )

        # show chats
        results = []
        for msg, meta in zip(chat_msgs.messages, chat_msgs.metas):
            msg = deserialise_message(msg)
            meta = json.loads(meta.pairs[0].second)
            timestamp = datetime.datetime.fromtimestamp(
                meta['timestamp']//1000000000, JST)
            # message = msg.message.argument_text or msg.message.text
            message = msg.result.response.query
            message_translate = self.translate(message, dest="en").text
            result = {'message': message,
                      'message_translate': message_translate,
                      'timestamp': timestamp,
                      'similarity': difflib.SequenceMatcher(
                          None, query, message_translate).ratio(),
                      'action': msg.result.response.action,
                      'msg': msg,
                      'meta': meta}
            if msg.result.response.action in ['make_reply', 'input.unknown']:
                rospy.logwarn(
                    "Found dialogflow messages {} at {}" +
                    "but skipping (action:{})".format(
                        result['message'],
                        result['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                        msg.result.response.action
                    ))
            else:
                rospy.logwarn(
                    "Found dialogflow messages {}({})({}) " +
                    "at {} ({}:{:.2f})".format(
                        result['message'],
                        result['message_translate'],
                        msg.result.response.action,
                        result['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                        query,
                        result['similarity']
                    ))
                if (result['similarity'] > threshold):
                    results.append(result)
                else:
                    rospy.logwarn(
                        "... skipping (threshold: {:.2f})".format(threshold))
        return results, chat_msgs

    def query_images_and_classify(self, query, start_time, end_time, limit=30):
        rospy.logwarn(
            "Query images from {} to {}".format(start_time, end_time))
        # meta_query= {'input_topic': '/spot/camera/hand_color/image/compressed/throttled',
        #              'inserted_at': {"$gt": start_time, "$lt": end_time}}
        meta_query = {
            'input_topic': '/head_camera/rgb/image_rect_color/compressed/throttled',
            'inserted_at': {"$gt": start_time, "$lt": end_time}
        }
        meta_tuple = (
            StringPair(
                MongoQueryMsgRequest.JSON_QUERY,
                json.dumps(meta_query, default=json_util.default)),
        )
        msgs = self.query(
            database='jsk_robot_lifelog',
            collection='fetch1075',
            type='sensor_msgs/CompressedImage',
            single=False,
            limit=limit,
            meta_query=StringPairList(meta_tuple),
            sort_query=StringPairList([StringPair('_meta.inserted_at', '-1')])
        )
        rospy.loginfo("Found {} images".format(len(msgs.messages)))
        if len(msgs.messages) == 0:
            rospy.logwarn("no images was found")

        # get contents of images
        results = []
        for msg, meta in zip(msgs.messages, msgs.metas):
            meta = json.loads(meta.pairs[0].second)
            timestamp = datetime.datetime.fromtimestamp(
                meta['timestamp']//1000000000, JST)
            # rospy.logwarn("Found images at {}".format(timestamp))

            goal = ClassificationTaskGoal()
            goal.compressed_image = deserialise_message(msg)
            goal.queries = [query]
            self.classification_ac.send_goal(goal)
            self.classification_ac.wait_for_result()
            result = self.classification_ac.get_result()
            idx = result.result.label_names.index(query)
            # similarities = result.result.probabilities
            similarities = result.result.label_proba
            rospy.logwarn(
                "... {}".format(
                    list(zip(result.result.label_names, map(
                        lambda x: "{:.2f}".format(x), similarities)
                    ))))
            rospy.logwarn("Found images at {} .. {}".format(
                timestamp,
                list(zip(result.result.label_names,
                         map(lambda x: "{:.4f}".format(x), similarities)))))
            results.append({'label': result.result.label_names[idx],
                            'probabilities': result.result.probabilities[idx],
                            'similarities': result.result.label_proba[idx],
                            'image': goal.compressed_image,
                            'timestamp': timestamp})
        # we do not sorty by probabilites,
        # becasue we also need oldest timestamp
        return results

    def publish_google_chat_card(self, text, filename=None):
        goal = SendMessageGoal()
        goal.text = text
        if filename:
            goal.cards = [Card(
                sections=[Section(widgets=[WidgetMarkup(
                    image=Image(localpath=filename))])])]
        goal.space = 'spaces/AAAAoTwLBL0'
        rospy.logwarn("send {} to {}".format(goal.text, goal.space))
        self.chat_ros_ac.send_goal_and_wait(
            goal, execute_timeout=rospy.Duration(0.10))

    def text_to_salience(self, text):
        goal = AnalyzeTextGoal()
        goal.text = text
        self.analyze_text_ac.send_goal(goal)
        self.analyze_text_ac.wait_for_result()
        entity = self.analyze_text_ac.get_result()
        if len(entity.entities) > 0:
            return entity.entities[0].name
        else:
            return text

    def translate(self, text, dest):
        return Translated(text=text,
                          dest=dest,
                          src="en",
                          origin="unknown",
                          pronunciation="unknown")
        global translator
        loop = 3
        while loop > 0:
            try:
                ret = translator.translate(text, dest="en")
                return ret
            except Exception as e:
                rospy.logwarn("Faile to translate {}".format(e))
                time.sleep(1)
                translator = Translator()
                loop = loop - 1
                return Translated(text=text, dest=dest)

    def cb(self, msg):
        if msg._type == 'google_chat_ros.msg/MessageEvent':
            # text = message.message.argument_text.lstrip() or \
            #     message.message.text.lstrip()
            text = msg.message.argument_text.lstrip() or \
                msg.message.text.lstrip()
            rospy.logwarn("Received chat message '{}'".format(text))

            # ask dialogflow for intent
            goal = DialogTextGoal()
            goal.query = text
            self.dialogflow_ac.send_goal(goal)
            self.dialogflow_ac.wait_for_result()
            result = self.dialogflow_ac.get_result()
        elif msg._type == 'dialogflow_task_executive/DialogTextActionResult':
            result = msg.result
        else:
            rospy.logerr("Unknown message type {}".format(msg._type))
            return

        try:
            rospy.logwarn(
                "received dialogflow query'{}'".format(result.response.query)
            )
            rospy.logwarn(
                "received dialogflow action'{}'".format(result.response.action)
            )
            print(result.response)
            if result.response.action == 'input.unknown':
                self.publish_google_chat_card("🤖")
            elif result.response.action == 'make_reply':
                translated = self.translate(result.response.query, dest="en")
                self.make_reply(translated.text, translated.src)
            else:
                self.publish_google_chat_card(result.response.response)
        except Exception as e:
            rospy.logerr(
                "Callback failed {} {}".format(e, traceback.format_exc()))
            self.publish_google_chat_card("💀 {}".format(e))


if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    ml = MessageListener()
    # ml.cb2(0)
    # ml.cb2('chair')
    rospy.spin()
