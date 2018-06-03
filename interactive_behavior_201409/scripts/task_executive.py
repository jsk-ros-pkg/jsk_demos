#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import defaultdict
import heapq
import itertools
import json
import re
import rospy

from app_manager.msg import AppList
from app_manager.srv import StartApp, StopApp
from std_msgs.msg import String
from interactive_behavior_201409.msg import Attention, DialogResponse
from interactive_behavior_201409.srv import EnqueueTask, EnqueueTaskResponse


def camel_to_snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


class AppManager(object):
    def __init__(self,
                 on_started=None,
                 on_stopped=None,
                 on_installed=None,
                 on_uninstalled=None,
                 timeout=30):
        # init variables
        self._latest_msg = None
        self._last_running = None
        self._last_available = None
        # init callbacks
        self._callbacks = {
            'started': on_started,
            'stopped': on_stopped,
            'installed': on_installed,
            'uninstalled': on_uninstalled,
        }
        # init interfaces
        ns = rospy.get_param("robot/name", "robot")
        self._srv_start_app = rospy.ServiceProxy(
            ns + "/start_app", StartApp)
        self._srv_stop_app = rospy.ServiceProxy(
            ns + "/stop_app", StopApp)
        self._sub_list_apps = rospy.Subscriber(
            ns + "/app_list", AppList, self._list_apps_cb)
        self._srv_start_app.wait_for_service(timeout=timeout)
        self._srv_stop_app.wait_for_service(timeout=timeout)
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self._latest_msg is not None:
                break
            rospy.loginfo("Waiting for apps")
            r.sleep()
        rospy.loginfo("AppManager initialized")

    def _list_apps_cb(self, msg):
        self._latest_msg = msg
        if self._last_running is not None and \
           self._last_available is not None:
            if self._callbacks["started"]:
                started = set(self.running_apps) - set(self._last_running)
                for name in started:
                    self._callbacks["started"](name)
            if self._callbacks["stopped"]:
                stopped = set(self._last_running) - set(self.running_apps)
                for name in stopped:
                    self._callbacks["stopped"](name)
            last_all = set(self._last_running) | set(self._last_available)
            all_apps = set(self.running_apps) | set(self.available_apps)
            if self._callbacks["installed"]:
                installed = all_apps - last_all
                for name in installed:
                    self._callbacks["installed"](name)
            if self._callbacks["uninstalled"]:
                uninstalled = last_all - all_apps
                for name in uninstalled:
                    self._callbacks["uninstalled"](name)
        self._last_running = self.running_apps
        self._last_available = self.available_apps

    @property
    def running_apps(self):
        return map(lambda a: a.name,
                   self._latest_msg.running_apps)

    @property
    def available_apps(self):
        return map(lambda a: a.name,
                   self._latest_msg.available_apps)

    def start_app(self, name):
        if name in self.running_apps:
            raise RuntimeError("%s is already running" % name)
        elif name not in self.available_apps:
            raise RuntimeError("%s is not available" % name)
        res = self._srv_start_app(name=name)
        if res.started:
            rospy.loginfo("%s successfully started" % name)
            return True
        else:
            raise RuntimeError("{0} failed to launch: {1} ({2})".format(
                name, res.message, res.error_code))

    def stop_app(self, name):
        if name not in self.running_apps:
            raise RuntimeError("%s is not running" % name)
        res = self._srv_stop_app(name=name)
        if res.stopped:
            rospy.loginfo("%s successfully stopped" % name)
            return True
        else:
            raise RuntimeError("{0} failed to stop: {1} ({2})".format(
                name, res.message, res.error_code))


class PriorityQueue(object):
    REMOVED = '<REMOVED>'

    def __init__(self, default_priority=5):
        self.heap = list()
        self.default_priority = default_priority
        self.mark_removed = dict()

    def push(self, element, priority=None):
        "Add a new element or update the priority of an existing element."
        if priority is None:
            priority = self.default_priority
        if element in self.mark_removed:
            self.remove(element)
        entry = [priority, element]
        self.mark_removed[element] = entry
        heapq.heappush(self.heap, entry)

    def pop(self):
        "Remove and return the highest priority element (1 is the highest)"
        while self.heap:
            _, element = heapq.heappop(self.heap)
            if element is not self.REMOVED:
                del self.mark_removed[element]
                return element
        raise ValueError('Empty queue')

    def remove(self, element):
        "Remove the element"
        entry = self.mark_removed.pop(element)
        entry[-1] = self.REMOVED

    def __len__(self):
        return len(self.heap)

    def __iter__(self):
        return self

    def next(self):
        try:
            return self.pop()
        except ValueError:
            raise StopIteration()


class TaskExecutive_old(object):
    def __init__(self):
        self.queue = PriorityQueue()
        self.current_task = set()
        self.idle_tasks = []
        self.attentions = defaultdict(list)
        #
        self.app_manager = AppManager(
            on_started=self.app_start_cb,
            on_stopped=self.app_stop_cb,
        )
        self.sub_attention = rospy.Subscriber(
            "~attention", Attention, self.attention_cb)
        self.srv_enqueue_task = rospy.Service(
            "~enqueue", EnqueueTask, self.enqueue_cb)

    def is_idle(self):
        running = self.active_tasks - set(self.idle_tasks)
        return not running

    def spawn_next_task(self):
        next_task = None
        # 1. check attention
        if self.attentions[Attention.LEVEL_IMPORTANT]:
            last_attention = self.attentions[Attention.LEVEL_IMPORTANT][-1]
        # 2. check queue
        # 3. check idle
        # 4. cancel current task if needed
        # 5. spawn next task
        if next_task is not None:
            if self.current_task:
                for t in self.current_task:
                    self.app_manager.stop_app(t)
            self.app_manager.start_app(next_task)

    def attention_cb(self, msg):
        rospy.loginfo("Attention")
        self.attentions[msg.level].append(msg)
        # filter duplicated attentions?
        self.spawn_next_task()

    def enqueue_cb(self, req):
        rospy.loginfo("Enqueue")
        res = EnqueueTaskResponse()
        self.queue.push(res.task)  # FIXME
        self.spawn_next_task()
        return res

    def app_start_cb(self, name):
        rospy.loginfo("%s started" % name)

    def app_stop_cb(self, name):
        rospy.loginfo("%s stopped" % name)
        self.spawn_next_task()


class TaskExecutive(object):
    def __init__(self):
        self.app_manager = AppManager(
            on_started=self.app_start_cb,
            on_stopped=self.app_stop_cb,
        )
        # load remappings
        self.action_remappings = rospy.get_param("~action_remappings", {})
        for key, app in self.action_remappings.items():
            if app not in self.app_manager.available_apps:
                rospy.logwarn("Action '%s' is not available")
                del self.action_remappings[key]

        self.sub_dialog = rospy.Subscriber(
            "dialog_response", DialogResponse,
            self.dialog_cb)

    @property
    def is_idle(self):
        return len(self.app_manager.running_apps) == 0

    def dialog_cb(self, msg):
        if not msg.action or msg.action.startswith('input.'):
            rospy.loginfo("Action '%s' is ignored" % msg.action)
            return
        if not self.is_idle:
            rospy.logerr("Action %s is already executing" % self.app_manager.running_apps)
            return
        # check extra action remappings
        if msg.action in self.action_remappings.values():
            action = msg.action
        elif msg.action in self.action_remappings:
            action = self.action_remappings[msg.action]
        else:
            action = "interactive_behavior_201409/" + camel_to_snake(msg.action)
        if action not in self.app_manager.available_apps:
            rospy.logerr("Action '%s' is unknown" % action)
            return
        try:
            params = json.loads(msg.parameters)
            rospy.set_param("/action/parameters", params)
        except ValueError:
            rospy.logerr("Failed to parse parameters of action '%s'" % msg.action)
            return
        rospy.loginfo("Starting '%s' with parameters '%s'" % (msg.action, msg.parameters))
        self.app_manager.start_app(action)

    def app_start_cb(self, name):
        rospy.loginfo("%s started" % name)

    def app_stop_cb(self, name):
        rospy.loginfo("%s stopped" % name)
        try:
            rospy.delete_param("/action/parameters")
            rospy.loginfo("Removed %s" % "/action/parameters")
        except KeyError:
            pass


def main():
    rospy.init_node("task_executive")
    e = TaskExecutive()
    rospy.spin()


if __name__ == '__main__':
    def test_priority_queue():
        priority = [1,3,5,7,9,2,100,104,1000,2000,999,999,999,0,45]
        task = range(len(priority))
        queue = PriorityQueue()
        for v in zip(priority, task):
            queue.push(v)
        print "pushed %d" % len(queue)
        for prio, task in queue:
            print prio, task

    def test_app_manager():
        rospy.init_node("test_app_manager")
        app = AppManager(
            on_started=lambda n: rospy.loginfo("started %s" % n),
            on_stopped=lambda n: rospy.loginfo("stopped %s" % n),
            on_installed=lambda n: rospy.loginfo("installed: %s" % n),
            on_uninstalled=lambda n: rospy.loginfo("uninstalled: %s" % n),
        )
        print "available:", app.available_apps
        print "running:", app.running_apps
        name = "jsk_pr2_startup/hello_world"
        print "starting:", name
        app.start_app(name)
        print "available:", app.available_apps
        print "running:", app.running_apps

    main()
