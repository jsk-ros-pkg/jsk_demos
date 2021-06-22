#! /usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PeoplePoseArray
from geometry_msgs.msg import PointStamped

class GetHumanPoseDepth:

    def __init__(self):
        self.sub = rospy.Subscriber("edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback)
        self.pub = rospy.Publisher ("head_camera/rgb/image_rect_color/screenpoint", PointStamped, queue_size=10)

    def callback(self, msg):
        rospy.loginfo_throttle_identical(10,"Received {} poses".format(len(msg.poses)))
        for pose in msg.poses:
            limb_name_list = pose.limb_names
            #print("nose" in limb_name_list)
            if ("nose" in limb_name_list):
                nose_number = limb_name_list.index("nose")
                nose_position = pose.poses[nose_number]
                nose_position_x = nose_position.position.x
                nose_position_y = nose_position.position.y
                rospy.logdebug_throttle_identical(10, "nose position {} {}".format(nose_position_x, nose_position_y))

                nose_pos_pointstamp = PointStamped()
                nose_pos_pointstamp.header.frame_id = "nose"
                nose_pos_pointstamp.point.x = nose_position_x
                nose_pos_pointstamp.point.y = nose_position_y
                nose_pos_pointstamp.point.z = 0.0

                rospy.loginfo_throttle_identical(10, "nose position {} {}".format(nose_pos_pointstamp.point.x, nose_pos_pointstamp.point.y))
                self.pub.publish(nose_pos_pointstamp)

def get_pose():
    rospy.init_node("get_human_pose_depth")
    get_pose = GetHumanPoseDepth()
    rospy.spin()

if __name__ == '__main__':
    get_pose()
