#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class QuestionGenerator:
    def __init__(self):
        rospy.init_node('question_generator')
        rospy.sleep(1.0)

        self.pub = rospy.Publisher('/input_text', String, queue_size=10)
        rospy.Subscriber('/barcode', String, self.callback)

        self.seen_questions = set()  # 既に処理済みのQR内容を記録

        rospy.loginfo("question_generatorノードが起動しました。/barcode を監視中...")
        rospy.spin()

    def callback(self, msg):
        question = msg.data.strip()

        # 条件1: 「？」または「?」で終わっている
        is_question = question.endswith('？') or question.endswith('?')

        # 条件2: 初めての文字列
        if is_question and question not in self.seen_questions:
            rospy.loginfo(f"新しい質問を検出: {question}")
            self.pub.publish(question)
            self.seen_questions.add(question)
        else:
            rospy.loginfo(f"無視: '{question}'（再検出または疑問文でない）")

if __name__ == '__main__':
    try:
        QuestionGenerator()
    except rospy.ROSInterruptException:
        pass
