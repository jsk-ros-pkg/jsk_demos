#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, math, time
from geometry_msgs.msg import Point

last = None
def cb(p: Point):
    global last
    if last is None:
        last = p; return
    dx, dy, dz = p.x - last.x, p.y - last.y, p.z - last.z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist > 1e-5:
        rospy.loginfo("paint_position: (%.3f,%.3f,%.3f) step=%.4f", p.x, p.y, p.z, dist)
    last = p

def main():
    rospy.init_node("watch_pose", anonymous=True)
    rospy.Subscriber("/paint_position", Point, cb, queue_size=50)
    rospy.spin()

if __name__ == "__main__":
    main()
