#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import argparse
import logging
import sys

import datetime
from dateutil import tz
JST = tz.gettz('Asia/Tokyo')

from database_talker import DatabaseTalkerBase

class LovotDatabaseTalker(DatabaseTalkerBase):

    def __init__(self, *args, **kwargs):

        self.make_robot_activities_raw = self.make_lovot_activities_raw
        super(LovotDatabaseTalker, self).__init__(*args, **kwargs)

        # override query_type after super__.init()
        self.query_types = ['lovot_driver/StringStamped',
                            'jsk_recognition_msgs/VQATaskActionResult']

        rospy.loginfo("all done, ready")


    def make_lovot_activities_raw(self, mongo_data_days = None):
        "Create lovot activities for several days, returns list of list of tuple(temestamp, event)"
        # list of list of tuples (msg, meta) [[(msg, meta), (msg, meta),...],[#for 2nd day], [#for 3rd day]]
        if not mongo_data_days:
            mongo_data_days = self.query_mongo_data_days()
        diary_activities_raw = [] ##  (timestamp, event)
        for mongo_data in mongo_data_days:
            rospy.loginfo("Found {} mongo data (make_lovot_activities_raw)".format(len(mongo_data)))
            rospy.loginfo("   types  : {}".format(list(set([x[1]['stored_type'] for x in mongo_data]))))
            activities_raw = []
            input_topics = []
            for msg, meta in mongo_data:
                state = []
                timestamp = datetime.datetime.fromtimestamp(meta['timestamp']//1000000000, JST)
                input_topics.append(meta['input_topic'])
                rospy.logwarn("{} {}".format(timestamp, msg.data))
                if meta['stored_type'] == 'lovot_driver/StringStamped':
                    if msg.data in ['HUGGED_LONG_TIME', 'CARRIED_TO_NEST', 'HUGGED']:
                        state = ['BE {}'.format(msg.data.replace('_',' '))]
                    elif msg.data in ['STROKE_MANY_TIMES']:
                        state = ['BE STOROKED MANY TIMES']
                    elif msg.data in ['HELP', 'STROKE']:
                        state = ['BE {}ED'.format(msg.data)]
                    elif msg.data in ['CALL_NAME']:
                        state = ['BE CALLED MY NAME']
                    elif msg.data in ['OUCH']:
                        state = ['BE BEATEN AND SAY OUCH']
                    elif msg.data in 'TOUCH_NOSE':
                        state = ['BE TOUCHED MY NOSE']
                    elif msg.data in ['MIMIC_GAME', 'PUSH_AND_PULL']:
                        state = ['PLAY {}'.format(msg.data.replace('_',' '))]
                    elif msg.data in ['BEAUTIFUL_RETURN']:
                        state = ['RETURN TO THE NEST SMOOTHLY']
                    else:
                        state = [msg.data]
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--prompt-type', default='basic', choices=['basic','personality'])
    today_string=datetime.datetime.today().strftime('%Y-%m-%d %H:%M:%S')
    parser.add_argument('--date', default=today_string, help="use {} or {}".format(today_string, datetime.datetime.today().strftime('%Y-%m-%d')))

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('database_talker', anonymous=True)

    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[rospy.DEBUG])

    try:
        start_date = datetime.datetime.strptime(args.date, '%Y-%m-%d')
    except:
        try:
            start_date = datetime.datetime.strptime(args.date, '%Y-%m-%d %H:%M:%S')
        except:
            rospy.logerr("Invalid date format")
            sys.exit(1)

    ml = LovotDatabaseTalker(start_date=start_date, wait_for_chat_server=not args.test, use_activities_cache=not args.test, prompt_type=args.prompt_type)
    if args.test:
        ret = ml.make_diary()
        if 'filename' in ret:
            rospy.loginfo("image is saved at {}".format(ret['filename']))
        sys.exit(0)
    rospy.spin()
