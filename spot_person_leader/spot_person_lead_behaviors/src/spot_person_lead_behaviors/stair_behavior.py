# -*- coding: utf-8 -*-

from spot_behavior_manager.base_behavior import BaseBehavior

import roslaunch
import rospkg
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray

class StairBehavior(BaseBehavior):

    def callback_visible(self, msg):

        if self.state_visible != msg.data:
            self.starttime_visibility = rospy.Time.now()
            self.duration_visibility = rospy.Duration()
            self.state_visible = msg.data
        else:
            self.duration_visibility = rospy.Time.now() - self.starttime_visibility

    def callback_people_pose_array(self, msg):

        self.exist_person_down = False
        for pose in msg.poses:
            if pose.position.z < -0.5:
                self.exist_person_down = True

    def run_initial(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_initial() called')

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_path = rospkg.RosPack().get_path('spot_person_lead_behaviors') +\
                          '/launch/stair_detection.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                                        uuid,
                                        roslaunch_file
                                        )
        self.roslaunch_parent.start()

        # value for person checker
        self.subscriber_visible = None
        self.state_visible = False
        self.starttime_visibility = rospy.Time.now()
        self.duration_visibility = rospy.Duration(10)

        # value for people pose array
        self.subscriber_people_pose_array = None
        self.msg_people_pose_array = None
        self.exist_person_down = False

        # start subscribers
        try:
            self.subscriber_visible = rospy.Subscriber(
                                        '/stair_detection_person_tracker/visible',
                                        Bool,
                                        self.callback_visible
                                        )
            self.subscriber_people_pose_array = rospy.Subscriber(
                                        '/stair_detection_person_tracker/people_pose_array',
                                        PoseArray,
                                        self.callback_people_pose_array
                                        )
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

        return True

    def run_main(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_main() called')

        graph_name = edge.properties['graph']
        start_id = filter(
                    lambda x: x['graph'] == graph_name,
                    start_node.properties['waypoints_on_graph']
                    )[0]['id']
        end_id = filter(
                    lambda x: x['graph'] == graph_name,
                    end_node.properties['waypoints_on_graph']
                    )[0]['id']
        localization_method = filter(
                    lambda x: x['graph'] == graph_name,
                    start_node.properties['waypoints_on_graph']
                    )[0]['localization_method']

        # graph uploading and localization
        if pre_edge is not None and \
            graph_name == pre_edge.properties['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            ret = self.spot_client.upload_graph(graph_name)
            if ret[0]:
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
            else:
                rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                return False
            # Localization
            if localization_method == 'fiducial':
                ret = self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                ret = self.spot_client.set_localization_waypoint(start_id)
            else:
                ret = (False,'Unknown localization method')
            if ret[0]:
                rospy.loginfo('robot is localized on the graph.')
            else:
                rospy.logwarn('Localization failed: {}'.format(ret[1]))
                return False

        # check if there is a person lower than the robot.
        while not rospy.is_shutdown():
            if self.exist_person_down:
                self.sound_client.say('私より低い位置に誰かいるのでいなくなるまで待ちます',blocking=True)
            else:
                break

        self.sound_client.say(
                '階段は危ないので私が昇り降りしている間は近づかないでください',
                blocking=True)

        # start leading
        self.spot_client.navigate_to( end_id, blocking=False)
        self._spot_client.wait_for_navigate_to_result()
        result = self._spot_client.get_navigate_to_result()

        # recovery on failure
        if not result.success:
            self.sound_client.say('失敗したので元に戻ります', blocking=True)
            self.spot_client.navigate_to( start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return result.success

    def run_final(self, start_node, end_node, edge, pre_edge ):

        rospy.logdebug('run_finalize() called')

        self.spot_client.cancel_navigate_to()

        if self.subscriber_visible != None:
            self.subscriber_visible.unregister()
            self.subscriber_visible = None

        if self.subscriber_people_pose_array != None:
            self.subscriber_people_pose_array.unregister()
            self.subscriber_people_pose_array = None

        self.roslaunch_parent.shutdown()
