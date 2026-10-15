#!/usr/bin/env python3

import rospy
import json
import os
import random
from std_msgs.msg import String, Float32


def main():
    rospy.init_node("propose_game_test_publisher")

    qr_pub = rospy.Publisher("/qr_data", String, queue_size=10)
    distance_pub = rospy.Publisher("/qr_distance", Float32, queue_size=10)

    base_dir = os.path.dirname(__file__)
    json_path = os.path.join(base_dir, "propose_game_word_card.json")

    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)

    selected = random.sample(data, 6)
    distance = 0.05

    rospy.sleep(1)

    rospy.loginfo("selected cards:")
    for card in selected:
        rospy.loginfo(f'{card["id"]}: {card["text"]}')

    for card in selected:
        distance_pub.publish(distance)
        rospy.sleep(0.2)

        qr_pub.publish(str(card["id"]))
        rospy.loginfo(f'published: qr_id={card["id"]}, text={card["text"]}, distance={distance}')

        rospy.sleep(1.0)

    rospy.loginfo("finished publishing 6 random QR codes")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
