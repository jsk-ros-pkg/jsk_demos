#!/usr/bin/env python

import rospy
import fysom
from std_srvs import srv
from drc_task_common.srv import *
from jsk_rviz_plugins.msg import OverlayText


# task state ###############################
def task_state_manager_init():
    # State machine
    global tsm
    tsm = fysom.Fysom({
        'initial': 'none',
        'events': [
            {'name': 'SolveIK', 'src': 'none', 'dst': 'SolveIK'},
            {'name': 'SuccessIK', 'src': 'none', 'dst': 'SuccessIK'},
            {'name': 'FailIK', 'src': 'none', 'dst': 'FailIK'},
            {'name': 'ExecGoPos', 'src': 'none', 'dst': 'ExecGoPos'},
            {'name': 'FinishGoPos', 'src': 'none', 'dst': 'FinishGoPos'},
            {'name': 'ExecAngleVec', 'src': 'none', 'dst': 'ExecAngleVec'},
            {'name': 'ExecReachUntilTouch', 'src': 'none', 'dst': 'ExecReachUntilTouch'},
            {'name': 'FinishAngleVec', 'src': 'none', 'dst': 'FinishAngleVec'},
            {'name': 'FinishReachUntilTouch', 'src': 'none', 'dst': 'FinishReachUntilTouch'},
        ]
    })
    # registar chnage state callback
    tsm.onchangestate = changeTaskState
    # add event to transit each state directly
    for dst in tsm.get_all_state():
        for src in tsm.get_all_state():
            tsm.add_event({'events':[{'name': dst, 'src': src, 'dst': dst}]})
    tsm.fatal = ['FailIK']
    tsm.warn = ['SolveIK', 'ExecGoPos', 'ExecAngleVec', 'ExecReachUntilTouch']

    # ROS
    global task_state_text_pub
    task_state_text_pub = rospy.Publisher("task_state_text", OverlayText)
    rospy.Service('call_task_state_event', StringRequest, callTaskStateEvent)

def changeTaskState(e):
    send_text = OverlayText()
    send_text.text = 'TaskState: '+e.dst
    send_text.top = 10
    send_text.left = 10
    send_text.width = 750
    send_text.height = 50

    send_text.bg_color.r = 0.9
    send_text.bg_color.b = 0.9
    send_text.bg_color.g = 0.9
    send_text.bg_color.a = 0.1
    if e.dst in tsm.fatal:
        send_text.fg_color.r = 0.8
        send_text.fg_color.g = 0.3
        send_text.fg_color.b = 0.3
    elif e.dst in tsm.warn:
        send_text.fg_color.r = 0.8
        send_text.fg_color.g = 0.5
        send_text.fg_color.b = 0.1
    else:
        send_text.fg_color.r = 0.3
        send_text.fg_color.g = 0.8
        send_text.fg_color.b = 0.3
    send_text.fg_color.a = 1
    send_text.line_width = 1
    send_text.text_size = 30
    task_state_text_pub.publish(send_text)

def callTaskStateEvent(req):
    global tsm
    getattr(tsm, req.data)()
    return StringRequestResponse()

# obj state ###############################
def obj_state_manager_init():
    # State machine
    global osm
    osm = fysom.Fysom({
        'initial': 'none',
        'events': [
            {'name': 'AutoSet', 'src': 'none', 'dst': 'AutoSet'},
            {'name': 'ManuSet', 'src': 'none', 'dst': 'ManuSet'},
            {'name': 'Assoc', 'src': 'none', 'dst': 'Assoc'},
        ]
    })
    # registar chnage state callback
    osm.onchangestate = changeObjState
    # add event to transit each state directly
    for dst in osm.get_all_state():
        for src in osm.get_all_state():
            osm.add_event({'events':[{'name': dst, 'src': src, 'dst': dst}]})
    osm.fatal = []
    osm.warn = []

    # ROS
    global obj_state_text_pub
    obj_state_text_pub = rospy.Publisher("obj_state_text", OverlayText)
    rospy.Service('call_obj_state_event', StringRequest, callObjStateEvent)

def changeObjState(e):
    send_text = OverlayText()
    send_text.text = 'ObjState: '+e.dst
    send_text.top = 70
    send_text.left = 10
    send_text.width = 750
    send_text.height = 50

    send_text.bg_color.r = 0.9
    send_text.bg_color.b = 0.9
    send_text.bg_color.g = 0.9
    send_text.bg_color.a = 0.1
    if e.dst in osm.fatal:
        send_text.fg_color.r = 0.8
        send_text.fg_color.g = 0.3
        send_text.fg_color.b = 0.3
    elif e.dst in osm.warn:
        send_text.fg_color.r = 0.8
        send_text.fg_color.g = 0.5
        send_text.fg_color.b = 0.1
    else:
        send_text.fg_color.r = 0.2
        send_text.fg_color.g = 0.7
        send_text.fg_color.b = 0.7
    send_text.fg_color.a = 1
    send_text.line_width = 1
    send_text.text_size = 30
    obj_state_text_pub.publish(send_text)

def callObjStateEvent(req):
    global osm
    getattr(osm, req.data)()
    return StringRequestResponse()

# extension of Fysom ###############################
## get_all_state
def get_all_state(self):
    def flatten(nested_list):
        flat_list = []
        fringe = [nested_list]
        while len(fringe) > 0:
            node = fringe.pop(0)
            if isinstance(node, list):
                fringe = node + fringe
            else:
                flat_list.append(node)
        return flat_list
    tmp = flatten(self._map.values())
    return list(set(flatten([[i.keys() for i in tmp], [i.values() for i in tmp]])))

## add_event
def add_event(self, cfg):
    events = cfg['events'] if 'events' in cfg else []
    callbacks = cfg['callbacks'] if 'callbacks' in cfg else {}
    tmap = self._map
    def add(e):
        src = [e['src']] if isinstance(e['src'], basestring) else e['src']
        if e['name'] not in tmap:
            tmap[e['name']] = {}
        for s in src:
            tmap[e['name']][s] = e['dst']
    for e in events:
        add(e)
    for name in tmap:
        setattr(self, name, self._build_event(name))
    for name in callbacks:
        setattr(self, name, callbacks[name])

# main ###############################
def state_manager_main():
    rospy.spin()

if __name__ == "__main__":
    setattr(fysom.Fysom, "get_all_state", get_all_state) 
    setattr(fysom.Fysom, "add_event", add_event) 
    rospy.init_node("task_state_manager")
    task_state_manager_init()
    # obj_state_manager_init()
    state_manager_main()
