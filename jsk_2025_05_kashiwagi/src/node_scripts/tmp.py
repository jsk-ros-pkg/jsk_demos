from sound_play.libsoundplay import SoundClient
import rospy
import sys, os, rospkg
from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest
rospy.init_node("kashiwagi_listener")

rospy.sleep(3.0)
client = SoundClient(sound_action='robotsound_jp', sound_topic='robotsound_jp', blocking=False)
rospy.sleep(3.0)

self.path_to_pkg = os.path.join(rospkg.RosPack().get_path("jsk_2025_05_kashiwagi"),)

client.playWave(
    f"{self.path_to_pkg}/data/kashiwagi_iiyo.wav"
)
