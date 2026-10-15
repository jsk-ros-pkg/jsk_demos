#!/usr/bin/env python3

import usb.core
import fcntl
import os
import signal
import subprocess
from pathlib import Path
from threading import Lock

import sensor_msgs.msg
import rosnode
import rosgraph
import psutil
import rospy
from nodelet.srv import NodeletList

from jsk_tools.sanity_lib import checkNodeState


try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy


def search_pid_from_process_cmd(cmd):
    pids = []
    for p in psutil.process_iter():
        if cmd in p.cmdline():
            pids.append(p.pid)
    return pids


def reset_usb(device_path):
    if device_path is None:
        rospy.logwarn('device_path is not exists. '
                      'Please set device_path')
        return False
    fd = os.open(device_path, os.O_WRONLY)
    if fd < 0:
        rospy.logerr("Could not open {}".format(device_path))
        return False
    rospy.loginfo("Resetting USB device")
    # Equivalent of the _IO('U', 20) constant in the linux kernel.
    USBDEVFS_RESET = ord('U') << (4*2) | 20
    try:
        rc = fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)


class CameraNodeletRespawner(object):

    def __init__(self):
        self.lock = Lock()

        self._nodelet_list = [
            '/stereo/camera_driver',
            # '/stereo/disparity',
            '/stereo/left_camera/rectify_color',
            # '/stereo/point_cloud',
            '/stereo/right_camera/rectify_color',
            '/stereo/split_image',
            '/stereo/throttle_image',
        ]

        dev = usb.core.find(idVendor=0x32e4, idProduct=0x9750)
        self.device = '/dev/bus/usb/{0:03d}/{1:03d}'.format(
            dev.bus, dev.address)

        self.img_msg = None
        self.sub = rospy.Subscriber('/stereo/image_raw',
                                    sensor_msgs.msg.Image,
                                    callback=self.callback,
                                    queue_size=1)

    def callback(self, msg):
        self.img_msg = msg

    def respawn(self):
        with self.lock:
            camera = 'stereo'
            if self._nodelet_list is None:
                nodelets = rospy.ServiceProxy(
                    '/{}/{}_nodelet_manager/list'.format(camera, camera),
                    NodeletList)().nodelets
            else:
                nodelets = self._nodelet_list

            manager_name = '{}_nodelet_manager'.format(camera)
            name = manager_name + '/respawner'
            master = rosgraph.Master(name)

            cmdlines = []
            for nodelet in nodelets:
                current_ns = Path(nodelet).parent
                tmp = rosnode.get_api_uri(
                    master, nodelet, skip_cache=True)

                if tmp is not None:
                    try:
                        pid = ServerProxy(tmp).getPid(name)[2]
                    except Exception as e:
                        print(e)
                        continue

                    tmp = psutil.Process(pid)
                    cmdline = tmp.cmdline()
                    cmdline.append("__ns:={}".format(current_ns))
                    print(cmdline)
                    for cc in cmdline:
                        if cc.startswith('__name:'):
                            cc = "__name:={}/{}".format(current_ns, cc)
                    cmdlines.append(cmdline)

                    rospy.loginfo("Killing nodelet: %s(%d)", nodelet, pid)
                    process = psutil.Process(pid)
                    if process.is_running():
                        process.send_signal(signal.SIGKILL)

                    start = rospy.Time.now()
                    while psutil.pid_exists(pid) and (
                            rospy.Time.now() - start < rospy.Duration(3.0)):
                        rospy.sleep(0.1)
                    if process.is_running():
                        process.send_signal(signal.SIGKILL)

            rospy.loginfo('Resetting usb')
            reset_usb(self.device)

            pids = search_pid_from_process_cmd('__name:={}'.format(manager_name))
            pid = pids[0]
            process = psutil.Process(pid)
            if process.is_running():
                process.send_signal(signal.SIGINT)
            else:
                raise Exception('manager process is not running')

            start = rospy.Time.now()
            while psutil.pid_exists(pid) and (
                    rospy.Time.now() - start < rospy.Duration(3.0)):
                rospy.sleep(0.1)
            if process.is_running():
                process.send_signal(signal.SIGKILL)

            manager_namespace = camera
            respawn_manager_cmd = ["rosrun", "nodelet",
                                   "nodelet", "manager",
                                   "__name:=" + manager_name,
                                   "__ns:=" + manager_namespace]
            child_process = subprocess.Popen(respawn_manager_cmd)

            name = '/{}/{}_nodelet_manager'.format(camera, camera)

            for cmdline in cmdlines:
                subprocess.Popen(cmdline)

            self.img_msg = None
            rospy.sleep(120.0)

    def run(self):
        rate = rospy.Rate(1)

        if self._nodelet_list is not None:
            while True:
                ret = []
                for node_name in self._nodelet_list:
                    ret.append(checkNodeState(node_name, needed=True))
                if all(ret):
                    break
                for node_name, alive in zip(self._nodelet_list, ret):
                    if alive is False:
                        rospy.logwarn('{} is not launched.'.format(node_name))
                rate.sleep()
        rospy.sleep(120.0)

        while not rospy.is_shutdown():
            rate.sleep()
            if self.img_msg is None:
                # rospy.logwarn('no image')
                pass
            else:
                duration = (rospy.Time.now()
                            - self.img_msg.header.stamp).to_sec()
                # rospy.logwarn('{}'.format(duration))
            if self.img_msg is None or duration > 120.0:
                rospy.logwarn('respawn camera nodes')
                self.respawn()


if __name__ == '__main__':
    rospy.init_node('camera_respawner')
    respawner = CameraNodeletRespawner()
    respawner.run()
