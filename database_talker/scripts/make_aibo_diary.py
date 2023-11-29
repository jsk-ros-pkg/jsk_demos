#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import logging

import argparse

import actionlib
from bson import json_util
# import copy
import cv2
import datetime
# import difflib
import json
import os
# import random
import pickle
import re
import random
import rospkg
# import shutil
import sys
# import yaml
import tempfile
# import time
import traceback

from dateutil import tz
JST = tz.gettz('Asia/Tokyo')

from cv_bridge import CvBridge
bridge = CvBridge()

# from googletrans import Translator
# from googletrans.models import Translated
# translator = Translator()

from mongodb_store.util import deserialise_message

from google_chat_ros.msg import Card, Section, WidgetMarkup, Image
from google_chat_ros.msg import MessageEvent, SendMessageAction, SendMessageGoal

from mongodb_store_msgs.msg import StringPairList, StringPair
from mongodb_store_msgs.srv import MongoQueryMsg, MongoQueryMsgRequest, MongoQueryMsgResponse

# from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal

# from dialogflow_task_executive.msg import DialogTextAction, DialogTextGoal, DialogTextActionResult

# from jsk_recognition_msgs.msg import ClassificationTaskAction, ClassificationTaskGoal
# from jsk_recognition_msgs.msg import VQATaskAction, VQATaskGoal

from openai_ros.srv import Completion, CompletionResponse

class MessageListener(object):

    def __init__(self, wait_for_chat_server=True):
        #self.pickle_file = tempfile.NamedTemporaryFile(suffix='.pickle')
        self.pickle_file = "/tmp/activities.pickle"
        self.robot_name = rospy.get_param('robot/name')
        rospy.loginfo("using '{}' database".format(self.robot_name))

        if self.robot_name == 'aibo':
            self.query_types = ['aibo_driver/StringStatus',
                                'aibo_driver/ObjectStatusArray',
                                'jsk_recognition_msgs/VQATaskActionResult']
        else:
            self.query_types = ['jsk_recognition_msgs/VQATaskActionResult']

        rospy.loginfo("wait for '/google_chat_ros/send'")
        self.chat_ros_ac = actionlib.SimpleActionClient('/google_chat_ros/send', SendMessageAction)
        if wait_for_chat_server:
            self.chat_ros_ac.wait_for_server()

        rospy.loginfo("wait for '/message_store/query_messages'")
        rospy.wait_for_service('/message_store/query_messages')
        self.query = rospy.ServiceProxy('/message_store/query_messages', MongoQueryMsg)

        # rospy.loginfo("wait for '/classification/inference_server'")
        # self.classification_ac = actionlib.SimpleActionClient('/classification/inference_server' , ClassificationTaskAction)
        # self.classification_ac.wait_for_server()

        # rospy.loginfo("wait for '/vqa/inference_server'")
        # self.vqa_ac = actionlib.SimpleActionClient('/vqa/inference_server' , VQATaskAction)
        # self.vqa_ac.wait_for_server()

        # # https://github.com/k-okada/openai_ros
        # # this requres apt install python3.7 python3.7-venv
        rospy.loginfo("wait for '/openai/get_response'")
        rospy.wait_for_service('/openai/get_response')
        self.completion = rospy.ServiceProxy('/openai/get_response', Completion)

        # ## integration of dialogflow <-> google_chat_ros was performed by google_chat_ros/script/helper.py
        # rospy.loginfo("wait for '/dialogflow_client/text_action'")
        # self.dialogflow_ac = actionlib.SimpleActionClient('/dialogflow_client/text_action' , DialogTextAction)
        # self.dialogflow_ac.wait_for_server()

        # rospy.loginfo("wait for '/analyze_text/text'")
        # self.analyze_text_ac = actionlib.SimpleActionClient('/analyze_text/text' , AnalyzeTextAction)
        # self.analyze_text_ac.wait_for_server()

        rospy.loginfo("subscribe '/google_chat_ros/message_activity'")
        self.sub = rospy.Subscriber('/google_chat_ros/message_activity', MessageEvent, self.cb)

        rospy.loginfo("all done, ready")

    def query_multiple_types(self, types, meta_tuple):
        "Query mongo messages, returns list of MongoQueryMsgResponse"
        msgs = MongoQueryMsgResponse()
        for _type in types:
            msg = self.query(database = 'jsk_robot_lifelog',
                             collection = self.robot_name,
                             type =  _type,
                             single = False,
                             # limit = limit,
                             meta_query = StringPairList(meta_tuple),
                             sort_query = StringPairList([StringPair('_meta.inserted_at', '-1')]))
            msgs.messages.extend(msg.messages)
            msgs.metas.extend(msg.metas)
        return msgs

    def query_mongo_data(self, types, start_time, end_time):
        "Query activities for aibo robot, returns list of tuple (msg, meta)"
        rospy.logwarn("Query activities from {} until {}".format(start_time, end_time))
        meta_query= {'inserted_at': {"$lt": end_time, "$gt": start_time}}
        meta_tuple = (StringPair(MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta_query, default=json_util.default)),)
        mongo_msgs = self.query_multiple_types(types, meta_tuple)

        activities = []
        for msg, meta in zip(mongo_msgs.messages, mongo_msgs.metas):
            msg = deserialise_message(msg)
            meta = json.loads(meta.pairs[0].second)
            activities.append((msg, meta))
        rospy.logwarn("  Found {} messages".format(len(activities)))
        return activities

    def query_mongo_data_days(self, types=None, days=7):
        "Query activities for a week, returns list of list of tuple (msg, meta), if activity is empty of that day, returns empty list"
        if types == None:
            types = self.query_types
        # if we found cache file
        if (os.path.exists(self.pickle_file) and
            (datetime.datetime.today() - datetime.datetime.fromtimestamp(os.path.getmtime(self.pickle_file))).seconds < 1 * 60 * 60):  # seconds -> hours
            rospy.loginfo('Loading cached activities data {}'.format(datetime.datetime.fromtimestamp(os.path.getmtime(self.pickle_file))))
            with open(self.pickle_file, 'rb') as f:
                return pickle.load(f)

        activities = []
        today = datetime.date.today()
        startdate = datetime.datetime(today.year, today.month, today.day, tzinfo=JST)
        for days_before in range(days):
            activities_raw = self.query_mongo_data(types,
                                                   startdate-datetime.timedelta(hours=days_before*24),
                                                   startdate-datetime.timedelta(hours=(days_before-1)*24))
            activities.append(activities_raw)

        # dump msgs
        with open(self.pickle_file, 'wb') as f:
            pickle.dump(activities, f)
            f.flush()

        return activities

    def make_aibo_activities_raw(self, mongo_data_days = None):
        "Create aibo activities for several days, returns list of list of tuple(temestamp, event)"
        # list of list of tuples (msg, meta) [[(msg, meta), (msg, meta),...],[#for 2nd day], [#for 3rd day]]
        if not mongo_data_days:
            mongo_data_days = self.query_mongo_data_days()
        diary_activities_raw = [] ##  (timestamp, event)
        for mongo_data in mongo_data_days:
            rospy.loginfo("Found {} mongo data (make_aibo_activities_raw)".format(len(mongo_data)))
            rospy.loginfo("   types  : {}".format(list(set([x[1]['stored_type'] for x in mongo_data]))))
            activities_raw = []
            input_topics = []
            for msg, meta in mongo_data:
                state = []
                timestamp = datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST)
                input_topics.append(meta['input_topic'])
                # rospy.logwarn("{} {}".format(timestamp, meta['input_topic']))
                if meta['stored_type'] == 'aibo_driver/StringStatus':
                    if msg.status in ['', 'none']:
                        continue
                    if 'body_touched' in meta['input_topic']:
                        state = [msg.status+' touched']
                    elif 'hungry' in meta['input_topic']:
                        state = ['energy is '+msg.status]
                    elif 'posture' in meta['input_topic']:
                        if msg.status in ['sleep']:
                            state = ['sleeping']
                        elif msg.status in ['stand']:
                            state = ['standing']
                        else:
                            state = [msg.status]
                    elif 'sleepy' in meta['input_topic']:
                        continue
                        #state = [msg.status]
                elif meta['stored_type'] == 'aibo_driver/ObjectStatusArray':
                    # remove duplicates from list https://stackoverflow.com/questions/7961363/removing-duplicates-in-lists
                    state = list(set(['found ' + state.name for state in msg.status]))
                elif meta['stored_type'] == 'jsk_recognition_msgs/VQATaskActionResult':
                    if len(msg.result.result.result) > 0:
                        answer = msg.result.result.result[0].answer
                        if len(answer.split()) > 3:
                            state = [answer]
                else:
                    continue
                # create activities_raw
                for s in state:
                    activities_raw.append({'timestamp': timestamp, 'state': s, 'type': meta['stored_type']})

            diary_activities_raw.append(activities_raw)

            if len(activities_raw) > 0:
                rospy.loginfo("   period : {} {}".format(activities_raw[-1]['timestamp'], activities_raw[0]['timestamp']))
                rospy.loginfo("   topics : {}".format({key: input_topics.count(key) for key in set(input_topics)}))
        ##
        return diary_activities_raw ##  (timestamp, event)

    def make_state_frequency(self, diary_activities_raw, message):
        diary_activities_freq = []
        for activities_raw in diary_activities_raw:
            activities_raw_state = [x['state'] for x in [x for x in activities_raw if message in x['type']]]
            activities_freq = {key: activities_raw_state.count(key) for key in set(activities_raw_state)}
            rospy.logwarn("Found {} activity data (make_state_frequency)".format(len(activities_raw)))
            if len(activities_raw) > 0:
                rospy.logwarn("   period : {} {}".format(activities_raw[-1]['timestamp'], activities_raw[0]['timestamp']))
                rospy.logwarn("     freq : {} ({})".format(activities_freq, len(activities_freq)))
            diary_activities_freq.append(activities_freq)
        return diary_activities_freq

    def make_activities_events(self, diary_activities_raw, message):
        diary_activities_events = []
        for activities_raw in diary_activities_raw:
            activities_events = {}
            for activities in activities_raw:
                timestamp = activities['timestamp']
                event = activities['state']
                if message not in activities['type']:
                    continue
                if event in activities_events:
                    time_since_last_seen = activities_events[event]['last_seen'] - timestamp
                    if time_since_last_seen.seconds/60 < 30:  # min
                        activities_events[event]['tmp_duration'] += time_since_last_seen
                    else:
                        # 'duration' keeps maximum duration
                        # if activities_events[event]['tmp_duration'] > activities_events[event]['duration']:
                        #     activities_events[event]['duration'] = activities_events[event]['tmp_duration']
                        # 'duration' keeps accumulated duration
                        activities_events[event]['duration'] += activities_events[event]['tmp_duration']
                        activities_events[event]['tmp_duration'] = datetime.timedelta()
                    activities_events[event]['last_seen'] = timestamp
                    activities_events[event]['count'] += 1
                else:
                    activities_events.update({event : {'last_seen' : timestamp, 'tmp_duration' : datetime.timedelta(), 'duration' : datetime.timedelta(), 'count': 0}})
                # print("{} {:24} {} {}".format(timestamp, event, activities_events[event]['duration'], activities_events[event]['tmp_duration']))
            diary_activities_events.append(activities_events)
        return diary_activities_events

    def make_image_activities(self, diary_activities_raw = None):
        if not diary_activities_raw:
            mongo_data_days = self.query_mongo_data_days()
            diary_activities_raw = self.make_aibo_activities_raw(mongo_data_days)

        # create activities event data
        # activities_events[event_name] = {'duration', datetime.timedelta, 'count': int}
        # diary_activities_events = [activities_events for day1, activities_events for day2, ....]
        diary_activities_events = self.make_activities_events(diary_activities_raw, 'jsk_recognition_msgs/VQATaskActionResult')

        image_activities = {}
        for activities_raw in diary_activities_raw:
            for activities in activities_raw:
                if activities['type'] != 'jsk_recognition_msgs/VQATaskActionResult':
                    continue
                timestamp = activities['timestamp']
                answer = activities['state']
                print(answer)
                if len(answer.split()) > 3 and answer not in image_activities.keys():
                    image_activities.update({answer : timestamp})
                if (len(image_activities)) > 0:
                    break
                else:
                    rospy.logwarn("   no valid image description is found...")
        #
        prompt = "Please select the most memorable and illuminating event by number from the list below.\n\n"
        n = 0
        for answer, timestamp in image_activities.items():
            prompt += "{}: {} ({})\n".format(n, answer, timestamp)
            n += 1

        response = self.openai_completion(prompt)
        n = re.search(r'(\d+)', response)
        if n:
            no = min(int(n.group(1)), len(image_activities)-1)
        else:
            no = random.randrange(len(image_activities))

        answer, timestamp = list(image_activities.items())[no]
        rospy.loginfo("topic of the day")
        rospy.loginfo("    answer : {}".format(answer))
        rospy.loginfo(" timestamp : {}".format(timestamp))
        results = self.query_images_and_classify(query = answer,
                                                 start_time = timestamp - datetime.timedelta(minutes=5),
                                                 end_time = timestamp + datetime.timedelta(minutes=5),
                                                 classify = False)
        if len(results) > 0:
            # pubish as card
            filename = tempfile.mktemp(suffix=".jpg", dir=rospkg.get_ros_home())
            self.write_image_with_annotation(filename, results[0], answer)
            return {'text': answer, 'filename': filename}

    def make_activity(self, diary_activities_raw = None):
        "Returns activity prompts"
        # create diary activities_raw
        # list of (timestamp, event)  [[{'temestamp': , 'state':, 'type': }, {'temestamp': , 'state':, 'type': } ...],[#for 2nd day],[#for 3rd day]...]
        if not diary_activities_raw:
            mongo_data_days = self.query_mongo_data_days()
            diary_activities_raw = self.make_aibo_activities_raw(mongo_data_days)

        # make frequencey data for 7days
        # activities_freq  {'event_1' : count, 'event_2' : count}
        # diary_activities_freq = [activities_freq for day1, activities_freq for day2, ...]
        diary_activities_freq = self.make_state_frequency(diary_activities_raw, 'aibo_driver/')

        # create activities event data
        # activities_events[event_name] = {'duration', datetime.timedelta, 'count': int}
        # diary_activities_events = [activities_events for day1, activities_events for day2, ....]
        diary_activities_events = self.make_activities_events(diary_activities_raw, 'aibo_driver/')

        for activities_events in diary_activities_events:
            print("--")
            for event, duration in sorted(activities_events.items(), key=lambda x: x[1]['duration'], reverse=True):
                print("{:24} : {:4.2f} min ({} times)".format(event, duration['duration'].seconds/60, duration['count']))

        # flatten list
        activities_events = [x for events in diary_activities_events for x in events.keys()]  # get all activities with duplicates

        # percentages of activities happend
        prompt = "{}\n\n".format(filter(None, diary_activities_events)[0].items()[0][1]['last_seen'].strftime("%a %d %b %Y"))
        prompt += "\n<actions you always do> 'action : time'\n"

        # sort activities event by it's occurence [list] -> sorted({key: count})
        activities_events_freq = sorted({key: activities_events.count(key) for key in set(activities_events)}.items(), key=lambda x:x[1], reverse=True)
        for event, count in activities_events_freq:
            if count/float(len(diary_activities_events)) > 0.5:
                prompt += "{} : {:.2f}\n".format(event, count/float(len(diary_activities_events)))

        # estimate frequence in 24h
        prompt += "\n<actions performed more compared to yesterday> 'action : increase from the number of time done yesterday'\n"

        more_yesterday_action = False
        diary_activities_events_no_empty = list(filter(None, diary_activities_events))
        if len(diary_activities_events_no_empty) >= 2:
            l0 = diary_activities_events_no_empty[0]
            l1 = diary_activities_events_no_empty[1]
            for event in set(activities_events):
                if event in l0 and event in l1:
                    increase = l0[event]['count'] - l1[event]['count']
                    if increase > 0:
                        prompt += "{} : +{}\n".format(event, increase)
                        more_yesterday_action = True
        if not more_yesterday_action:
            prompt += "none\n"

        #
        prompt += "\n<actions happend after a long time> 'action : number of days passed since you last did it'\n"
        long_time_action = False
        for event in diary_activities_events[0].keys():
            n = 1
            for diary_activities_event in diary_activities_events[1:]:
                if event not in diary_activities_event.keys() or diary_activities_event[event]['duration'].seconds < 1:
                    n += 1
                else:
                    break
            if n >= 2:
                prompt += "{} : {}\n".format(event, n)
                long_time_action = True
        if not long_time_action:
            prompt += "none\n"

        rospy.logdebug(prompt)
        return prompt

    def make_diary(self, language="Japanese"):
        "make dirary"
        # get mongo data for 7 days
        mongo_data_days = self.query_mongo_data_days()
        diary_activities_raw = self.make_aibo_activities_raw(mongo_data_days)
        # get most impressive image and text
        topic_of_day = None
        _filename = False

        image_activity = self.make_image_activities(diary_activities_raw)
        if image_activity:
            topic_of_day = image_activity['text']
            filename = image_activity['filename']

        # create prompt
        prompt = "You are a baby robot. You were taken care of by people around you."
        if topic_of_day:
            prompt = "Today, you are impressed by " + topic_of_day + "."
        prompt += "The following data is a record of today's actions regarding what we always do, what we did more than yesterday, and what happened after a long time. Please write a brief diary from the data. Note, however, that you are a baby robot, so please make it a child-like diary.\n\n"

        prompt +=  self.make_activity(diary_activities_raw)

        response = self.openai_completion(prompt)
        rospy.loginfo("prompt = {}".format(prompt))
        rospy.loginfo("response = {}".format(response))

        prompt = "Please rewrite the following diary in {language}. Write as childlike as you can. Write a maximum 120 {language} charactors.\n\n".format(language = language) + response
        # prompt = "Please rewrite the following diary as childlike as you can. Write a maximum 120 {} charactors.\n\n".format(language) + response
        response = self.openai_completion(prompt)
        rospy.loginfo("prompt = {}".format(prompt))
        rospy.loginfo("response = {}".format(response))
        return {'text': response, 'filename': filename}

    def make_response(self, text, language="Japanese"):
        if language=="Japanese":
            text = self.openai_completion('Translate the following sentences to English "{}"'.format(text))
        prompt = "You are a baby robot. You were taken care of by people around you. The following data is a record of today's actions regarding what we always do, what we did more of yesterday, and What happened after a long time.\nIf your frined ask you as '{}', wow do you reply? Note, however, that you are a baby robot, so please make it a child-like response.\n\n".format(text) + self.make_activity()

        response = self.openai_completion(prompt)
        rospy.loginfo("prompt = {}".format(prompt))
        rospy.loginfo("response = {}".format(response))

        prompt = "Please rewrite the following response as childlike as you can. Write a maximum 120 {} charactors.\n\n".format(language) + response
        response = self.openai_completion(prompt)
        rospy.loginfo("prompt = {}".format(prompt))
        rospy.loginfo("response = {}".format(response))
        return {'text': response}

    def make_reply(self, message, lang="en", startdate=datetime.datetime.now(JST)-datetime.timedelta(hours=24), duration=datetime.timedelta(hours=24) ):
        enddate = startdate+duration
        rospy.logwarn("Run make_reply({} from {} to {})".format(message, startdate, enddate))
        query = self.text_to_salience(message)
        rospy.logwarn("query using salience word '{}'".format(query))
        # look for images
        try:
            # get chat message
            results, chat_msgs = self.query_dialogflow(query, startdate, enddate, threshold=0.25)
            # retry = 0
            # while retry < 3 and len(results) == 0 and len(chat_msgs.metas) > 0:
            #     meta = json.loads(chat_msgs.metas[-1].pairs[0].second)
            #     results, chat_msgs = self.query_dialogflow(query, datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST))
            #     retry = retry + 1
            # sort based on similarity with 'query'
            chat_msgs_sorted = sorted(results, key=lambda x: x['similarity'], reverse=True)

            if len(chat_msgs_sorted) == 0:
                rospy.logwarn("no chat message was found")
            else:
                # query images that was taken when chat_msgs are stored
                msg = chat_msgs_sorted[0]['msg']
                meta = chat_msgs_sorted[0]['meta']
                text = chat_msgs_sorted[0]['message']
                startdate = chat_msgs_sorted[0]['timestamp']
                action = chat_msgs_sorted[0]['action']
                similarity = chat_msgs_sorted[0]['similarity']
                # query chat to get response
                #meta = json.loads(chat_msgs_sorted[0]['meta'].pairs[0].second)
                # text = msg.message.argument_text or msg.message.text
                # startdate = datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST)
                rospy.loginfo("Found message '{}'({}) at {}, corresponds to query '{}' with {:2f}%".format(text, action, startdate.strftime('%Y-%m-%d %H:%M:%S'), query, similarity))

            # query images when chat was received
            start_time = startdate  # startdate is updated with found chat space
            end_time = enddate  # enddate is not modified within this function, it is given from chat
            results = self.query_images_and_classify(query=query, start_time=start_time, end_time=end_time)

            # no images found
            if len(results) == 0:
                return {'text': '記憶がありません🤯'}

            end_time = results[-1]['timestamp']

            # sort
            results = sorted(results, key=lambda x: x['similarities'], reverse=True)
            rospy.loginfo("Probabilities of all images {}".format(list(map(lambda x: (x['timestamp'].strftime('%Y-%m-%d %H:%M:%S'), x['similarities']), results))))
            best_result = results[0]

            '''
            # if probability is too low, try again
            while len(results) > 0 and results[0]['similarities'] < 0.25:

                start_time = end_time-datetime.timedelta(hours=24)
                timestamp = datetime.datetime.now(JST)
                results = self.query_images_and_classify(query=query, start_time=start_time, end_time=end_time, limit=300)
                if len(results) > 0:
                    end_time = results[-1]['timestamp']
                    # sort
                    results = sorted(results, key=lambda x: x['similarities'], reverse=True)
                    #rospy.loginfo("Probabilities of all images {}".format(list(map(lambda x: (x['label'], x['similarities']), results))))
                    if len(results) > 0 and results[0]['similarities'] > best_result['similarities']:
                        best_result = results[0]

            rospy.loginfo("Found '{}' image with {:0.2f} % simiarity at {}".format(best_result['label'], best_result['similarities'], best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')))
            '''

            ## make prompt
            reaction = self.describe_image_scene(best_result['image'])
            if len(chat_msgs_sorted) > 0 and chat_msgs_sorted[0]['action'] and 'action' in chat_msgs_sorted[0]:
                reaction += " and you felt " + chat_msgs_sorted[0]['action']
            rospy.loginfo("reaction = {}".format(reaction))

            # make prompt
            prompt = 'if you are a pet and someone tells you \"' + message + '\" when we went together, ' + \
                     'and ' + reaction + ' in your memory of that moment, what would you reply? '+ \
                     'Show only the reply in {lang}'.format(lang={'en': 'English', 'ja':'Japanese'}[lang])
            loop = 0
            result = None
            while loop < 3 and result is None:
                try:
                    result = self.completion(prompt=prompt,temperature=0)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s"%e)
                    result = None
                loop += 1
            result.text = result.text.lstrip().encode('utf8')
            rospy.loginfo("prompt = {}".format(prompt))
            rospy.loginfo("result = {}".format(result))
            # pubish as card
            filename = tempfile.mktemp(suffix=".jpg", dir=rospkg.get_ros_home())
            self.write_image_with_annotation(filename, best_result, prompt)
            return {'text': result.text, 'filename': filename}

        except Exception as e:
            raise ValueError("Query failed {} {}".format(e, traceback.format_exc()))


    def openai_completion(self, prompt, temperature=0):
        loop = 0
        result = None
        while loop < 5 and result is None:
            try:
                result = self.completion(prompt=prompt,temperature=temperature)
                if result.text == '':
                    rospy.logwarn(result)
                    rospy.logwarn("result text is too short, retry completion")
                    rospy.sleep(2)
                    resut = None
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s"%e)
                rospy.sleep(2)
                result = None
            loop += 1
        if result is None:
            raise Exception('[ERROR] openni_completion failed to complete {}'.format(prompt))
        result.text = result.text.lstrip()
        rospy.logdebug("prompt = {}".format(prompt))
        rospy.logdebug("result = {}".format(result))
        return result.text

    def write_image_with_annotation(self, filename, best_result, prompt):
        image = bridge.compressed_imgmsg_to_cv2(best_result['image'])
        _, width, _ = image.shape
        scale = width/640.0
        if 'label' in best_result and 'similarities' in best_result:
            cv2.putText(image, "{} ({:.2f}) {}".format(best_result['label'], best_result['similarities'], best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')),
                        (10,int(20*scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5*scale, (255,255,255), 8, 1)
            cv2.putText(image, "{} ({:.2f}) {}".format(best_result['label'], best_result['similarities'], best_result['timestamp'].strftime('%Y-%m-%d %H:%M:%S')),
                        (10,int(20*scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5*scale, (0,0,0), 2, 1)
        string_width = 70
        for i in range(0, len(prompt), string_width):  # https://stackoverflow.com/questions/13673060/split-string-into-strings-by-length
            text = prompt[i:i+string_width]
            cv2.putText(image, text, (10,int(43*scale)+int(i/string_width*20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5*scale, (255,255,255), 4, 1)
            cv2.putText(image, text, (10,int(43*scale)+int(i/string_width*20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5*scale, (0,0,0), 1, 1)
        cv2.imwrite(filename, image)
        rospy.logwarn("save images to {}".format(filename))


    def query_dialogflow(self, query, start_time, end_time, limit=30, threshold=0.0):
        rospy.logwarn("Query dialogflow from {} until {}".format(start_time, end_time))
        meta_query= {'inserted_at': {"$lt": end_time, "$gt": start_time}}
        meta_tuple = (StringPair(MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta_query, default=json_util.default)),)
        chat_msgs = self.query(database = 'jsk_robot_lifelog',
                               collection = self.robot_name,
                               # type =  'google_chat_ros/MessageEvent',
                               type =  'dialogflow_task_executive/DialogTextActionResult',
                               single = False,
                               # limit = limit,
                               meta_query = StringPairList(meta_tuple),
                               sort_query = StringPairList([StringPair('_meta.inserted_at', '-1')]))

        # optimization... send translate once
        messages = ''
        for msg, meta in zip(chat_msgs.messages, chat_msgs.metas):
            msg = deserialise_message(msg)
            message = msg.result.response.query.replace('\n','')
            messages += message + '\n'
        messages = self.translate(messages, dest="en").text.split('\n')

        # show chats
        results = []
        for msg, meta in zip(chat_msgs.messages, chat_msgs.metas):
            msg = deserialise_message(msg)
            meta = json.loads(meta.pairs[0].second)
            timestamp = datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST)
            # message = msg.message.argument_text or msg.message.text
            message = msg.result.response.query
            #message_translate = self.translate(message, dest="en").text
            message_translate = messages.pop(0).strip()
            result = {'message': message,
                      'message_translate': message_translate,
                      'timestamp': timestamp,
                      'similarity': difflib.SequenceMatcher(None, query, message_translate).ratio(),
                      'action': msg.result.response.action,
                      'msg': msg,
                      'meta': meta}
            if msg.result.response.action in ['make_reply', 'input.unknown']:
                rospy.logwarn("Found dialogflow messages {}({}) at {} but skipping (action:{})".format(result['message'], result['message_translate'], result['timestamp'].strftime('%Y-%m-%d %H:%M:%S'), msg.result.response.action))
            else:
                rospy.loginfo("Found dialogflow messages {}({}) ({}) at {} ({}:{:.2f})".format(result['message'], result['message_translate'], msg.result.response.action, result['timestamp'].strftime('%Y-%m-%d %H:%M:%S'), query, result['similarity']))
                if ( result['similarity'] > threshold):
                    results.append(result)
                else:
                    rospy.logwarn("                    ... skipping (threshold: {:.2f})".format(threshold))


        return results, chat_msgs


    def query_images_and_classify(self, query, start_time, end_time, limit=10, classify=True):
        rospy.logwarn("Query images from {} to {}".format(start_time, end_time))
        meta_query= {#'input_topic': '/spot/camera/hand_color/image/compressed/throttled',
                      'inserted_at': {"$gt": start_time, "$lt": end_time}}
        meta_tuple = (StringPair(MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta_query, default=json_util.default)),)
        msgs = self.query(database = 'jsk_robot_lifelog',
                          collection =  self.robot_name,
                          type =  'sensor_msgs/CompressedImage',
                          single = False,
                          limit = limit,
                          meta_query = StringPairList(meta_tuple),
                          sort_query = StringPairList([StringPair('_meta.inserted_at', '-1')]))

        rospy.loginfo("Found {} images".format(len(msgs.messages)))
        if len(msgs.messages) == 0:
            rospy.logwarn("no images was found")

        # get contents of images
        results = []
        for msg, meta in zip(msgs.messages, msgs.metas):
            meta = json.loads(meta.pairs[0].second)
            timestamp = datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST)
            # rospy.logwarn("Found images at {}".format(timestamp))

            result = {'query' : query, 'image' : deserialise_message(msg), 'timestamp': timestamp}
            if classify:
                goal = ClassificationTaskGoal()
                goal.compressed_image = result['image']
                goal.queries = [query]
                self.classification_ac.send_goal(goal)
                self.classification_ac.wait_for_result()
                result = self.classification_ac.get_result()
                idx =  result.result.label_names.index(query)
                #similarities = result.result.probabilities
                similarities = result.result.label_proba
                # rospy.logwarn("                ... {}".format(list(zip(result.result.label_names, map(lambda x: "{:.2f}".format(x), similarities)))))
                rospy.logwarn("Found images at {} .. {}".format(timestamp, list(zip(result.result.label_names, map(lambda x: "{:.4f}".format(x), similarities)))))
                result.update({'label': result.result.label_names[idx], 'probabilities': result.result.probabilities[idx], 'similarities': result.result.label_proba[idx]})
            results.append(result)

        # we do not sorty by probabilites, becasue we also need oldest timestamp
        return results

    def describe_image_scene(self, image):
        goal = VQATaskGoal()
        goal.compressed_image = image

        # unusual objects
        if random.randint(0,1) == 1:
            goal.questions = ['what unusual things can be seen?']
            reaction = 'you saw '
        else:
            goal.questions = ['what is the atmosphere of this place?']
            reaction = 'the atmosphere of the scene was '

        # get vqa result
        self.vqa_ac.send_goal(goal)
        self.vqa_ac.wait_for_result()
        result = self.vqa_ac.get_result()
        reaction += result.result.result[0].answer
        return reaction

    def publish_google_chat_card(self, text, space, filename=None):
        goal = SendMessageGoal()
        goal.text = text
        if filename:
            goal.cards = [Card(sections=[Section(widgets=[WidgetMarkup(image=Image(localpath=filename))])])]
        goal.space = space
        rospy.logwarn("send {} to {}".format(goal.text, goal.space))
        self.chat_ros_ac.send_goal_and_wait(goal, execute_timeout=rospy.Duration(0.10))

    def text_to_salience(self, text):
        goal = AnalyzeTextGoal()
        goal.text = text;
        self.analyze_text_ac.send_goal(goal)
        self.analyze_text_ac.wait_for_result()
        entity = self.analyze_text_ac.get_result()
        if len(entity.entities) > 0:
            return entity.entities[0].name
        else:
            return text

    def translate(self, text, dest):
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
        space = 'spaces/AAAAoTwLBL0' ## default space JskRobotBot
        if msg._type == 'google_chat_ros/MessageEvent':
            text = msg.message.argument_text.lstrip() or msg.message.text.lstrip()
            space = msg.space.name
            rospy.logwarn("Received chat message '{}'".format(text))
        else:
            rospy.logerr("Unknown message type {}".format(msg._type))
            return

        try:
            language = 'English' if text.isascii() else 'Japanese'
            if any(x in text for x in ['diary', '日記']):
                self.publish_google_chat_card("Sure!", space)
                ret = self.make_diary(language)
                if 'filename' in ret:
                    # upload text first, then upload images
                    self.publish_google_chat_card(ret['text'], space)
                    self.publish_google_chat_card('', space, ret['filename'])
                else:
                    self.publish_google_chat_card(ret['text'], space)
            else:
                ret = self.make_response(text, language)
                self.publish_google_chat_card(ret['text'], space)

        except Exception as e:
            rospy.logerr("Callback failed {} {}".format(e, traceback.format_exc()))
            self.publish_google_chat_card("💀 {}".format(e), space)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    args = parser.parse_args()

    rospy.init_node('test', anonymous=True)

    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[rospy.DEBUG])

    ml = MessageListener(wait_for_chat_server=not args.test)
    if args.test:
        ret = ml.make_diary()
        rospy.loginfo("image is saved at {}".format(ret['filename']))
        sys.exit(0)
    rospy.spin()
