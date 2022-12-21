#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('nearest_pos', Float32MultiArray, queue_size=1)

def callback(msg):
    boxes = msg.boxes

    # there is no person around the camera
    if (not boxes):
        rospy.loginfo("no person")
        return -1

    length = len(boxes)
    min_distance = 10000
    min_ind = 0

    # identify the person closest to the camera 
    for i in range(length):
        position = boxes[i].pose.position
        distance = position.x*position.x +position.y*position.y
        if (min_distance > distance):
            min_distance = distance
            min_ind = i

    coordinate = [boxes[min_ind].pose.position.x, boxes[min_ind].pose.position.y, boxes[min_ind].pose.position.z]

    position_number = 100

    if ((-0.6 <= coordinate[0]) and (coordinate[0] < 1.5)):
       if ((0.3 < coordinate[1]) and (coordinate[1] < 0.6)):
           position_number = 1 # left
       elif ((-0.6 < coordinate[1]) and (coordinate[1] < -0.3)):
           position_number = 2 # right

    min_position = []
    min_position.append(position_number)
    min_position.append(coordinate[0])
    min_position.append(coordinate[1])
    min_position.append(coordinate[2])
    min_position.append(min_distance)

    min_forPublish = Float32MultiArray(data=min_position)
    pub.publish(min_forPublish)
    
def listener():
    rospy.Subscriber("/rect_array_in_panorama_to_bounding_box_array/bbox_array", BoundingBoxArray, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('distance', anonymous=True)
    listener()
