#!/usr/bin/env python3
import rospy, random
from techrie_demo.msg import DesireState

class Desire:
    def __init__(self, v=0.2, rise=0.01, decay=0.003):
        self.v, self.rise, self.decay = v, rise, decay
    def tick(self, boost=0.0, damp=0.0):
        self.v = max(0.0, min(1.0, self.v + self.rise + boost - (self.decay + damp)))

def main():
    rospy.init_node('desire_manager')
    pub = rospy.Publisher('/desire/state', DesireState, queue_size=10)
    paint = Desire(0.3, 0.006, 0.002); with_people = Desire(0.4, 0.005, 0.0025)
    show = Desire(0.2, 0.004, 0.002); eat = Desire(0.1, 0.002, 0.0015); idle = Desire(0.2, 0.003, 0.001)
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        paint.tick(boost=0.01 if random.random()<0.03 else 0.0)
        with_people.tick(boost=0.01)
        show.tick(); eat.tick(); idle.tick()
        pub.publish(DesireState(paint.v, with_people.v, show.v, eat.v, idle.v))
        r.sleep()

if __name__=='__main__':
    main()
