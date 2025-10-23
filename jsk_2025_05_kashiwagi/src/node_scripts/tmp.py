from sound_play.libsoundplay import SoundClient
import rospy
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
rospy.init_node("kashiwagi_listener")

rospy.sleep(3.0)
client = SoundClient(sound_action='robotsound_jp', sound_topic='robotsound_jp', blocking=False)
rospy.sleep(3.0)

client.playWave(
    "/home/ubuntu/ros/kashiwagi_ws/src/jsk_demos/jsk_2025_05_kashiwagi/data/kashiwagi_iiyo.wav"
)
