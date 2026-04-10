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

    ml = DatabaseTalkerBase(start_date=start_date, wait_for_chat_server=not args.test, use_activities_cache=not args.test, prompt_type=args.prompt_type)
    if args.test:
        ret = ml.make_diary()
        if 'filename' in ret:
            rospy.loginfo("image is saved at {}".format(ret['filename']))
        sys.exit(0)
    rospy.spin()
