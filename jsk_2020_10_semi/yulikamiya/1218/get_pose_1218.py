#! /usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PeoplePoseArray
from geometry_msgs.msg import PointStamped

def callback(msg):
    limb_name_list = msg.poses[0].limb_names
    print("nose" in limb_name_list)
    if ("nose" in limb_name_list):
        nose_number = limb_name_list.index("nose")
        nose_position = msg.poses[0].poses[nose_number]
        nose_position_x = nose_position.position.x
        nose_position_y = nose_position.position.y
        print(nose_position)

        nose_pos_publisher = rospy.Publisher ("camera/color/image_raw/screenpoint", PointStamped, queue_size=10)
        nose_pos_pointstamp = PointStamped()
        nose_pos_pointstamp.header.frame_id = "nose"
        nose_pos_pointstamp.point.x = nose_position_x
        nose_pos_pointstamp.point.y = nose_position_y
        nose_pos_pointstamp.point.z = 0.0

        nose_pos_publisher.publish(nose_pos_pointstamp)

    print("Position"+str(msg.poses[0].limb_names))

def get_pose():
    rospy.init_node("get_pose")
    rospy.Subscriber("edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, callback)
    rospy.spin()

if __name__ == '__main__':
    get_pose()