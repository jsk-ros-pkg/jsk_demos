import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

rospy.init_node('talker_node')
sound_client = SoundClient(sound_action="/robotsound_jp", blocking=True)


rospy.sleep(1)

text = "今日はいい天気だね"
sound_client.say(text, voice='ja')
print("I said こんにちは")

text = "hello"
sound_client.say(text)
print("I said hello")

rospy.spin()
