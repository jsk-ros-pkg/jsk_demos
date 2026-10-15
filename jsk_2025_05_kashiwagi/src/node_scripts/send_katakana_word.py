#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("katakana_word_sender")
    pub = rospy.Publisher("/katakana_theme_word", String, queue_size=10)

    rospy.loginfo("Type a word and press Enter. (Ctrl+C to exit)")
    
    # rospy.Rate は必要なら使う（今回は不要）
    while not rospy.is_shutdown():
        try:
            # Python の標準入力
            text = input("> ").strip()
            if text == "":
                continue

            msg = String(data=text)
            pub.publish(msg)
            rospy.loginfo(f"Published: {text}")

        except (EOFError, KeyboardInterrupt):
            rospy.loginfo("Exiting katakana_word_sender.")
            break

if __name__ == "__main__":
    main()
