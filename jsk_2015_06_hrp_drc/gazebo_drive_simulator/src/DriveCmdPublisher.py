#! /usr/bin/env python
# license removed for brevity
import time
import sys
import rospy
from std_msgs.msg import Int8, Float64

class drive_command_publisher:
    def __init__(self):
        # If you change following obstacle_input topic name, you should also change topic name in handle_controller.launch
        rospy.Subscriber("brake_pedal/obstacle/input", Float64, self.callback_obstacle_stop)
        self.argv = sys.argv
        self.argc = len(self.argv)
        if self.argc == 3:
            self.time_flag = False
        elif self.argc == 4:
            self.time_flag = True
        else:
            print("The Number of Command-Line Arguments Is Unsuitable")
            print("Usage: ./DriveCmdPublisher.py TYPE DATA (TIME)")
            sys.exit(1)

        # argv[1] is the type of topic 
        if self.argv[1] in {"a", "b", "d", "h"}:
            self.pub_finish =  rospy.Publisher("/drc_vehicle_xp900/brake_pedal/cmd", Float64, queue_size=10)
            if self.argv[1] == "a":
                self.pub = rospy.Publisher("/drc_vehicle_xp900/gas_pedal/cmd", Float64, queue_size=10)
                rospy.init_node("drive_acceleration_publisher", anonymous=True)
            elif self.argv[1] == "b":
                self.pub = rospy.Publisher("/drc_vehicle_xp900/brake_pedal/cmd", Float64, queue_size=10)
                rospy.init_node("drive_brake_publisher", anonymous=True)
            elif self.argv[1] == "d":
                self.pub = rospy.Publisher("/drc_vehicle_xp900/direction/cmd", Int8, queue_size=10)
                rospy.init_node("drive_direction_publisher", anonymous=True)
            elif self.argv[1] == "h":
                rospy.init_node("drive_steering_publisher", anonymous=True)
                self.pub = rospy.Publisher("/drc_vehicle_xp900/hand_wheel/cmd", Float64, queue_size=10)
        else:
            print("TYPE(1st Argument) Means \"a\"(Accelerator Pedal), \"b\"(Brake Pedal), \"d\"(Direction Gear) or \"h\" (Steering Wheel)")
            sys.exit(1)

        # argv[2] is the data the topic has
        try:
            self.data = float(self.argv[2])
        except:
            print("DATA(2nd Argument) Should Be Float Type")
            sys.exit(1)

        # argv[3] is the time
        if self.time_flag:
            try:
                self.pub_time = float(self.argv[3])
            except:
                print("TIME(3rd Argument) Should Be Float Type")
                sys.exit(1)

        self.r = rospy.Rate(30) # 30hz

        # time of start publisher
        self.pub_start = time.time()

    def execute(self):
        while not rospy.is_shutdown():
            self.cmd_publisher()
            self.r.sleep()

    def cmd_publisher(self):
        if self.argv[1] != "d":
            # calculate publishing time
            current_time = time.time() - self.pub_start
            if self.time_flag:
                if current_time < self.pub_time:
                    pub_msg = Float64()
                    pub_msg = self.data
                    self.pub.publish(pub_msg)
                elif current_time < (self.pub_time + 1):
                    pub_msg = Float64()
                    pub_msg = 1.0
                    self.pub_finish.publish(pub_msg)
                    if self.argv[1] != "h":
                        pub_msg = Float64()
                        pub_msg = 0.0
                        self.pub.publish(pub_msg)
                elif current_time < (self.pub_time + 1.5):
                    pub_msg = Float64()
                    pub_msg = 0.0
                    self.pub_finish.publish(pub_msg)
                else:
                    print("Publish %s For %f Seconds & Publish b For 1.5 Seconds" % (self.argv[1], self.pub_time))
                    sys.exit(0)
            else:
                pub_msg = Float64()
                pub_msg = self.data
                self.pub.publish(pub_msg)
        else:
            pub_msg = Int8()
            if (self.data >= 0):
                pub_msg = 1
            else:
                pub_msg = -1
            self.pub.publish(pub_msg)
            

    def callback_obstacle_stop(self, msg):
        if self.argv[1] != "h" and self.argv[1] != "d":
            pub_msg = Float64()
            pub_msg = 0.0
            self.pub.publish(pub_msg)
        print("Obstacle Detection Activated!!")
        sys.exit(0)

if __name__ == '__main__':
    try:
        node = drive_command_publisher()
        node.execute()
    except rospy.ROSInterruptException: pass
