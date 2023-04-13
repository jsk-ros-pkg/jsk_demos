#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import datetime
import difflib
import json
import os
import shutil
import tempfile

import actionlib
import cv2
import rospkg
import rospy
from bson import json_util
from dateutil import tz

JST = tz.gettz('Asia/Tokyo')

from cv_bridge import CvBridge

bridge = CvBridge()

from googletrans import Translator

translator = Translator()

from dialogflow_task_executive.msg import (DialogTextAction,
                                           DialogTextActionResult,
                                           DialogTextGoal)
from google_chat_ros.msg import (Card, Image, MessageEvent, Section,
                                 SendMessageAction, SendMessageGoal,
                                 WidgetMarkup)
from jsk_recognition_msgs.msg import (ClassificationTaskAction,
                                      ClassificationTaskGoal)
from mongodb_store.util import deserialise_message
from mongodb_store_msgs.msg import StringPair, StringPairList
from mongodb_store_msgs.srv import MongoQueryMsg, MongoQueryMsgRequest
from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal


class MessageListener(object):

  def __init__(self):
    rospy.loginfo("wait for '/google_chat_ros/send'")
    self.chat_ros_ac = actionlib.SimpleActionClient('/google_chat_ros/send',
                                                    SendMessageAction)
    self.chat_ros_ac.wait_for_server()
    #self.pub = rospy.Publisher('/google_chat_ros/send/goal', SendMessageActionGoal, queue_size=1)

    rospy.loginfo("wait for '/message_store/query_messages'")
    rospy.wait_for_service('/message_store/query_messages')
    self.query = rospy.ServiceProxy('/message_store/query_messages',
                                    MongoQueryMsg)

    rospy.loginfo("wait for '/classification/clip_server'")
    self.classification_ac = actionlib.SimpleActionClient(
        '/classification/clip_server', ClassificationTaskAction)
    self.classification_ac.wait_for_server()

    ## integration of dialogflow <-> google_chat_ros was performed by google_chat_ros/script/helper.py
    # rospy.loginfo("wait for '/dialogflow_client/text_action'")
    # self.dialogflow_ac = actionlib.SimpleActionClient('/dialogflow_client/text_action' , DialogTextAction)
    # self.dialogflow_ac.wait_for_server()

    rospy.loginfo("wait for '/analyze_text/text'")
    self.analyze_text_ac = actionlib.SimpleActionClient('/analyze_text/text',
                                                        AnalyzeTextAction)
    self.analyze_text_ac.wait_for_server()

    # rospy.loginfo("subscribe '/google_chat_ros/message_activity'")
    # self.sub = rospy.Subscriber('/google_chat_ros/message_activity', MessageEvent, self.cb)
    rospy.loginfo("subscribe '/dialogflow_client/text_action/result'")
    self.sub = rospy.Subscriber('/dialogflow_client/text_action/result',
                                DialogTextActionResult, self.cb)

    rospy.loginfo("all done, ready")

  def make_reply(self, query):
    rospy.logwarn("Run make_reply({})".format(query))
    # look for images
    try:
      # get chat message
      results, chat_msgs = self.query_chat(query, datetime.datetime.now(JST))
      if len(results) == 0 and len(chat_msgs.metas) > 0:
        meta = json.loads(chat_msgs.metas[-1].pairs[0].second)
        results, chat_msgs = self.query_chat(
            query,
            datetime.datetime.fromtimestamp(meta['timestamp'] // 1000000000,
                                            JST))
      # sort based on similarity with 'query'
      chat_msgs_sorted = sorted(results,
                                key=lambda x: x['similarity'],
                                reverse=True)

      if len(chat_msgs_sorted) == 0:
        rospy.logwarn("no chat message was found")
        return
      else:
        # query images that was taken when chat_msgs are stored
        msg = chat_msgs_sorted[0]['msg']
        meta = chat_msgs_sorted[0]['meta']
        text = chat_msgs_sorted[0]['message']
        timestamp = chat_msgs_sorted[0]['timestamp']
        #meta = json.loads(chat_msgs_sorted[0]['meta'].pairs[0].second)
        # text = msg.message.argument_text or msg.message.text
        # timestamp = datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST)
        rospy.logwarn(
            "Found message '{}' at {}, corresponds to query {}".format(
                text, timestamp.strftime('%Y-%m-%d %H:%M:%S'), query))

      start_time = timestamp - datetime.timedelta(minutes=30)
      end_time = timestamp + datetime.timedelta(minutes=30)
      results = self.query_images_and_classify(query=query,
                                               start_time=start_time,
                                               end_time=end_time)

      end_time = results[-1]['timestamp']
      # sort
      results = sorted(results, key=lambda x: x['similarities'], reverse=True)
      rospy.loginfo("Probabilities of all images {}".format(
          list(map(lambda x: (x['label'], x['similarities']), results))))
      best_result = results[0]

      # if probability is too low, try again
      while len(results) > 0 and results[0]['similarities'] < 0.25:
        start_time = end_time - datetime.timedelta(hours=24)
        timestamp = datetime.datetime.now(JST)
        results = self.query_images_and_classify(query=query,
                                                 start_time=start_time,
                                                 end_time=end_time,
                                                 limit=300)
        if len(results) > 0:
          end_time = results[-1]['timestamp']
          # sort
          results = sorted(results,
                           key=lambda x: x['similarities'],
                           reverse=True)
          #rospy.loginfo("Probabilities of all images {}".format(list(map(lambda x: (x['label'], x['similarities']), results))))
          if len(results) > 0 and results[0]['similarities'] > best_result[
              'similarities']:
            best_result = results[0]

      rospy.loginfo("Found '{}' image with {:0.2f} % simiarity at {}".format(
          best_result['label'], best_result['similarities'],
          best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')))
      filename = tempfile.mktemp(suffix=".jpg", dir=rospkg.get_ros_home())
      image = bridge.compressed_imgmsg_to_cv2(best_result['image'])
      cv2.putText(
          image, "{} ({:.2f}) {}".format(
              best_result['label'], best_result['similarities'],
              best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')), (10, 20),
          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 8, 1)
      cv2.putText(
          image, "{} ({:.2f}) {}".format(
              best_result['label'], best_result['similarities'],
              best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')), (10, 20),
          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, 1)
      cv2.imwrite(filename, image)
      rospy.logwarn("save images to {}".format(filename))

      # pubish as card
      self.publish_google_chat_card(
          translator.translate('We saw ' + query, dest="ja").text, filename)

    except Exception as e:
      rospy.logerr("Query failed {}".format(e))

  def query_chat(self, query, end_time, limit=30):
    rospy.logwarn("Query chat until {}".format(end_time))
    meta_query = {'inserted_at': {"$lt": end_time}}
    meta_tuple = (StringPair(MongoQueryMsgRequest.JSON_QUERY,
                             json.dumps(meta_query,
                                        default=json_util.default)),)
    chat_msgs = self.query(
        database='jsk_robot_lifelog',
        collection='strelka',
        # type =  'google_chat_ros/MessageEvent',
        type='dialogflow_task_executive/DialogTextActionResult',
        single=False,
        limit=limit,
        meta_query=StringPairList(meta_tuple),
        sort_query=StringPairList([StringPair('_meta.inserted_at', '-1')]))

    # show chats
    results = []
    for msg, meta in zip(chat_msgs.messages, chat_msgs.metas):
      msg = deserialise_message(msg)
      meta = json.loads(meta.pairs[0].second)
      timestamp = datetime.datetime.fromtimestamp(
          meta['timestamp'] // 1000000000, JST)
      # message = msg.message.argument_text or msg.message.text
      message = msg.result.response.query
      result = {
          'message': message,
          'timestamp': timestamp,
          'similarity': difflib.SequenceMatcher(None, query, message).ratio(),
          'msg': msg,
          'meta': meta
      }
      if msg.result.response.action in ['make_reply', 'input.unknown']:
        rospy.logwarn(
            "Found chat messages {} at {} but skipping (action:{})".format(
                result['message'],
                result['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                msg.result.response.action))
      else:
        results.append(result)
        rospy.logwarn("Found chat messages {} at {} ({}:{:.2f})".format(
            result['message'],
            result['timestamp'].strftime('%Y-%m-%d %H:%M:%S'), query,
            result['similarity']))

    return results, chat_msgs

  def query_images_and_classify(self, query, start_time, end_time, limit=30):
    rospy.logwarn("Query images from {} to {}".format(start_time, end_time))
    meta_query = {
        'input_topic': '/spot/camera/hand_color/image/compressed/throttled',
        'inserted_at': {
            "$gt": start_time,
            "$lt": end_time
        }
    }
    meta_tuple = (StringPair(MongoQueryMsgRequest.JSON_QUERY,
                             json.dumps(meta_query,
                                        default=json_util.default)),)
    msgs = self.query(database='jsk_robot_lifelog',
                      collection='strelka',
                      type='sensor_msgs/CompressedImage',
                      single=False,
                      limit=limit,
                      meta_query=StringPairList(meta_tuple),
                      sort_query=StringPairList(
                          [StringPair('_meta.inserted_at', '-1')]))

    rospy.loginfo("Found {} images".format(len(msgs.messages)))
    if len(msgs.messages) == 0:
      rospy.logwarn("no images was found")

    # get contents of images
    results = []
    for msg, meta in zip(msgs.messages, msgs.metas):
      meta = json.loads(meta.pairs[0].second)
      timestamp = datetime.datetime.fromtimestamp(
          meta['timestamp'] // 1000000000, JST)
      # rospy.logwarn("Found images at {}".format(timestamp))

      goal = ClassificationTaskGoal()
      goal.compressed_image = deserialise_message(msg)
      goal.queries = [query]
      self.classification_ac.send_goal(goal)
      self.classification_ac.wait_for_result()
      result = self.classification_ac.get_result()
      idx = result.result.label_names.index(query)
      #similarities = result.result.probabilities
      similarities = result.result.label_proba
      # rospy.logwarn("                ... {}".format(list(zip(result.result.label_names, map(lambda x: "{:.2f}".format(x), similarities)))))
      rospy.logwarn("Found images at {} .. {}".format(
          timestamp,
          list(
              zip(result.result.label_names,
                  map(lambda x: "{:.2f}".format(x), similarities)))))
      results.append({
          'label': result.result.label_names[idx],
          'probabilities': result.result.probabilities[idx],
          'similarities': result.result.label_proba[idx],
          'image': goal.compressed_image,
          'timestamp': timestamp
      })

    # we do not sorty by probabilites, becasue we also need oldest timestamp
    return results

  def publish_google_chat_card(self, text, filename=None):
    goal = SendMessageGoal()
    goal.text = text
    if filename:
      goal.cards = [
          Card(sections=[
              Section(widgets=[WidgetMarkup(image=Image(localpath=filename))])
          ])
      ]
    goal.space = 'spaces/AAAAoTwLBL0'
    rospy.logwarn("send {} to {}".format(goal.text, goal.space))
    self.chat_ros_ac.send_goal_and_wait(goal,
                                        execute_timeout=rospy.Duration(0.10))

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

  def cb(self, msg):
    if msg._type == 'google_chat_ros.msg/MessageEvent':
      text = message.message.argument_text.lstrip(
      ) or message.message.text.lstrip()
      message = self.text_to_salience(text)
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
      rospy.logwarn("received dialogflow query  '{}'".format(
          result.response.query))
      rospy.logwarn("received dialogflow action '{}'".format(
          result.response.action))
      if result.response.action == 'input.unknown':
        self.publish_google_chat_card("🤖")
      elif result.response.action == 'make_reply':
        self.make_reply(
            self.text_to_salience(
                translator.translate(result.response.query, dest="en").text))
      else:
        self.publish_google_chat_card(result.response.response)

    except Exception as e:
      rospy.logerr("Callback failed {}".format(e))
      self.publish_google_chat_card("💀 {}".format(e))


if __name__ == '__main__':
  rospy.init_node('test', anonymous=True)
  ml = MessageListener()
  #ml.cb2(0)
  #ml.cb2('chair')
  rospy.spin()
