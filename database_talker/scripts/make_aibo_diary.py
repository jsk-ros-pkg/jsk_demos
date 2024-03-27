#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import argparse
import logging
import sys

from database_talker import DatabaseTalkerBase

class MessageListener(DatabaseTalkerBase):

    def __init__(self, *args, **kwargs):

        self.query_types = ['aibo_driver/StringStatus',
                            'aibo_driver/ObjectStatusArray',
                            'jsk_recognition_msgs/VQATaskActionResult']

        self.make_robot_activities_raw = self.make_aibo_activities_raw
        super(MessageListener, self).__init__(*args, **kwargs)
        rospy.loginfo("all done, ready")


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
                elif meta['stored_type'] == 'spot_msgs/Feedback':
                    state = []
                    if msg.standing:
                        state.append("standing")
                    if msg.sitting:
                        state.append("sitting")
                    if msg.moving:
                        state.append("moving")
                elif meta['stored_type'] == 'spot_msgs/ManipulatorState':
                    state = []
                    if msg.is_gripper_holding_item:
                        state.append("holding_item")
                elif meta['stored_type'] == 'jsk_recognition_msgs/VQATaskActionResult':
                    if len(msg.result.result.result) > 0:
                        answer = msg.result.result.result[0].answer
                        if len(answer.split()) > 3:
                            state = [answer]
                else:
                    rospy.logwarn("Unknown stored type: {}".format(meta['stored_type']))
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


    def make_diary(self, language="Japanese"):
        "make dirary"
        # get mongo data for 7 days
        mongo_data_days = self.query_mongo_data_days()
        diary_activities_raw = self.make_aibo_activities_raw(mongo_data_days)
        # get most impressive image and text
        topic_of_day = None
        filename = False

        image_activity = self.make_image_activities(diary_activities_raw)
        if image_activity:
            topic_of_day = image_activity['text']
            filename = image_activity['filename']

        # create prompt
        if self.prompt_type == 'personality':
            # from Ichikura's comment on 2024/Jan/23
            prompt = "<Assistant>\nYou are a pet robot, aibo. Your name is 'wasabi.'\nYou are shy. Your bithday is Dec. 25th, 2018. You are aware to be called your name and run into the voice. You like playing with your pink  ball  very much. You like being pampered by people. You are so polite to your owner. You like interacting with people. If you are hungry, you can go back to your charge station by yourself. You have 12 aibo friends. \n\nPlease write a brief diary of today from the data. Note, however, that you are a baby robot, so please write today's diary as simply and childishly as possible.\n\n<Prompt>\n"

        else:
            prompt = "You are a baby robot. You were taken care of by people around you.\n\n"

        if topic_of_day:
            prompt += "Today, you are impressed by " + topic_of_day + "."
        prompt += "The following data is a record of today's actions regarding what we always do, what we did more than yesterday, and what happened after a long time. Please write a brief diary from the data. Note, however, that you are a baby robot, so please make it a child-like diary.\n\n"

        prompt +=  self.make_activity(diary_activities_raw)

        response = self.openai_completion(prompt)
        rospy.loginfo("prompt = {}".format(prompt))
        rospy.loginfo("response = {}".format(response))

        prompt = "Please rewrite the following diary in {language}. Write as childlike as you can. Write around 360 {language} charactors.\n\n".format(language = language) + response
        # prompt = "Please rewrite the following diary as childlike as you can. Write a maximum 120 {} charactors.\n\n".format(language) + response
        response_short = self.openai_completion(prompt)
        rospy.loginfo("prompt = {}".format(prompt))
        rospy.loginfo("response = {}".format(response_short))
        if len(response_short) > 100:
            response = response_short
        else:
            rospy.logerr("response is too short ({} chars), use original version".format(len(response_short)))

        response = {'text': response}
        if filename:
            response.update({'filename': filename})

        return response


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--prompt-type', default='basic', choices=['basic','personality'])

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('test', anonymous=True)

    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[rospy.DEBUG])

    ml = MessageListener(wait_for_chat_server=not args.test, prompt_type=args.prompt_type)
    if args.test:
        ret = ml.make_diary()
        if 'filename' in ret:
            rospy.loginfo("image is saved at {}".format(ret['filename']))
        sys.exit(0)
    rospy.spin()
