#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy
import message_filters
import PyKDL

import tf2_ros
import tf2_geometry_msgs

import math
import threading

from sound_play.libsoundplay import SoundClient

from sensor_msgs.msg import Image, PanoramaInfo
from jsk_recognition_msgs.msg import RectArray, ClassificationResult
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import Label, LabelArray
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, PoseStamped
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse

def convert_msg_point_to_kdl_vector(point):
    return PyKDL.Vector(point.x,point.y,point.z)

class TrackedObject(object):

    UNKNOWN = 0
    FOUND = 1
    MOVING = 2
    APPROCHING = 3
    LOST = 4

    def __init__(self,
                 label,
                 position,
                 observed_time
                 ):

        self.label = label
        self.state = self.FOUND
        self.position = position
        self.velocity = PyKDL.Vector()
        self.last_observed = observed_time
        self.lock = threading.Lock()

    def checkLost(self,
                  observed_time,
                  threshold_lost):
        
        with self.lock:
        
            if (observed_time - self.last_observed).to_sec() > threshold_lost:
                self.state = self.LOST
                return True
            else:
                return False

    def update(self,
               observed_position,
               observed_time,
               robot_position_fixedbased,
               threshold_move,
               threshold_angle,
               threshold_distance):

        with self.lock:

            if observed_time < self.last_observed:
                rospy.logerr('observed time is earlier than last observed time, dropped.')
                return
            elif observed_time == self.last_observed:
                rospy.loginfo('this object is already observed')
                return

            # Update position and velocity
            if self.state == self.FOUND or self.state == self.MOVING or self.state == self.APPROCHING:
                self.velocity = (observed_position - self.position) / (observed_time - self.last_observed).to_sec()
            else:
                self.velocity = PyKDL.Vector()
            self.position = observed_position
            self.last_observed = observed_time

            # Update State
            if (self.state == self.LOST or self.state == self.UNKNOWN) or (self.velocity.Norm() < threshold_move):
                self.state = self.FOUND
            else:
                distance_obj2robot = (robot_position_fixedbased - self.position).Norm()
                direction_obj2robot = (robot_position_fixedbased - self.position)/distance_obj2robot
                angle_diff = math.acos(PyKDL.dot(self.velocity, direction_obj2robot) / self.velocity.Norm())

                if angle_diff < threshold_angle and distance_obj2robot > threshold_distance:
                    self.state = self.APPROCHING
                else:
                    self.state = self.MOVING

    def getDistance(self, robot_position):
        return (self.position - robot_position).Norm()

class Demo(object):

    def __init__(self):

        self._frame_fixed = rospy.get_param('~frame_fixed', 'fixed_frame')
        self._frame_robot = rospy.get_param('~frame_robot', 'base_link')
        self._duration_timeout = rospy.get_param('~duration_timeout', 0.05)

        # parameters for multi object trackers
        ## maximum number of tracking objects
        self._num_max_track = rospy.get_param('~num_max_track', 10)
        ## threshold for MOT state update
        self._thresholds_distance = rospy.get_param('~thresholds_distance', {})
        self._threshold_angle = rospy.get_param('~threshold_angle', 0.8)
        self._threshold_lost = rospy.get_param('~threshold_lost_duration', 1.0)
        self._threshold_move = rospy.get_param('~threshold_move_velocity', 1.0)
        ##
        self._threshold_tracking_switching_distance = rospy.get_param('~threshold_tracking_switching_distance',0.5)
        ##
        self._threshold_target_close_distance = rospy.get_param('~threshold_target_close_distance', 0.5)
        ##
        self._destination_offset_from_target_x = rospy.get_param('~destination_offset_from_target_x', 0.0)
        self._destination_offset_from_target_y = rospy.get_param('~destination_offset_from_target_y', 0.0)
        self._destination_offset_from_target_theta = rospy.get_param('~destination_offset_from_target_theta', 0.0)
        ##
        self._max_vx = rospy.get_param('~max_vx', 1.0)
        self._max_vy = rospy.get_param('~max_vy', 0.5)
        self._max_vtheta = rospy.get_param('~max_vtheta', 1.0)

        # command move
        self._use_trajectory = rospy.get_param('~use_trajectory', True)
        self._rate_control = rospy.get_param('~rate_control', 10.0)

        # members
        self._lock_for_dict_objects = threading.RLock()
        self._dict_objects = {}

        # sound client
        self._sound_client = SoundClient(sound_action='robotsound', sound_topic='robotsound')
        self._sound_client_jp = SoundClient(sound_action='robotsound_jp', sound_topic='robotsound_jp')

        # tf client
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # action client
        self._action_client_go_pos = actionlib.SimpleActionClient(
                                        '~go_pos',
                                        TrajectoryAction
                                        )
        self._action_client_go_pos.wait_for_server()

        # publishers
        self._pub_cmd_vel = rospy.Publisher('~cmd_vel',Twist,queue_size=1)
        self._pub_destination = rospy.Publisher('~destination',PoseStamped,queue_size=1)
        self._pub_twist_command = rospy.Publisher('~twist_command',TwistStamped,queue_size=1)

        # Subscriber
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message('~input_bbox_array', BoundingBoxArray, 3)
                rospy.wait_for_message('~input_tracking_labels', LabelArray, 3)
                break
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logwarn(
                    'subscribing topic seems not to be pulished. waiting... Error: {}'.format(e))
            rate.sleep()
        mf_sub_bbbox_array = message_filters.Subscriber('~input_bbox_array', BoundingBoxArray)
        mf_sub_tracking_labels = message_filters.Subscriber('~input_tracking_labels', LabelArray)
        ts = message_filters.TimeSynchronizer([mf_sub_bbbox_array, mf_sub_tracking_labels], 10)
        ts.registerCallback(self._cb_object)

        # service server
        self._flag_server = False
        self._lock_for_flag_server = threading.Lock()
        self._server_trigger = rospy.Service('~follow_person',Trigger,self._handler)

        rospy.loginfo('Node is successfully initialized')

    def _cb_object(self,
                   msg_bbox_array,
                   msg_tracking_labels):

        rospy.loginfo('callback called')

        if msg_bbox_array.header.frame_id != self._frame_fixed:
            rospy.logerr('frame_id of bbox (which is {}) array must be the same `~frame_fixed` which is {}'.format(msg_bbox_array.header.frame_id,self._frame_fixed))
            return

        time_observed = msg_tracking_labels.header.stamp

        try:
            pykdl_transform_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self._frame_fixed,
                    self._frame_robot,
                    time_observed,
                    timeout=rospy.Duration(self._duration_timeout)
                )
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup transform failed. {}'.format(e))
            return

        with self._lock_for_dict_objects:
            # add new object and update existing object state
            for bbox, tracking_label in zip(msg_bbox_array.boxes, msg_tracking_labels.labels):

                if tracking_label.id not in self._dict_objects:
                    if len(self._dict_objects) < self._num_max_track:
                        self._dict_objects[tracking_label.id] = \
                            TrackedObject(
                                bbox.label,
                                convert_msg_point_to_kdl_vector(bbox.pose.position),
                                time_observed
                            )
                    else:
                        rospy.logwarn('number of objects exceeds max track. dropped.')
                else:
                    self._dict_objects[tracking_label.id].update(
                                convert_msg_point_to_kdl_vector(bbox.pose.position),
                                time_observed,
                                pykdl_transform_fixed_to_robot.p,
                                self._threshold_move,
                                self._threshold_angle,
                                self._thresholds_distance[str(bbox.label)]
                    )
            # check if there is lost object
            to_be_removed = []
            for key in self._dict_objects:
                is_lost = self._dict_objects[key].checkLost(
                                time_observed,
                                self._threshold_lost
                                )
                if is_lost:
                    to_be_removed.append(key)
            # remove lost object from dict
            for key in to_be_removed:
                self._dict_objects.pop(key)

    def _handler(self,req):

        with self._lock_for_flag_server:
            self._flag_server = not self._flag_server
        return TriggerResponse(True,'Succeeded')

    def getNearestTargetID(self):
    
        distance = 3.0 # 3.0m以内
        target_id = None

        try:
            pykdl_transform_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                self._tf_buffer.lookup_transform(
                    self._frame_fixed,
                    self._frame_robot,
                    rospy.Time(),
                    timeout=rospy.Duration(self._duration_timeout)
                )
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return None

        with self._lock_for_dict_objects:
            for key in self._dict_objects:
                position_robotbased = pykdl_transform_fixed_to_robot.Inverse() * self._dict_objects[key].position
                if distance == None and target_id == None:
                    target_id = key
                    distance = position_robotbased.Norm()
                else:
                    if position_robotbased.Norm() < distance:
                        target_id = key
                        distance = position_robotbased.Norm()
        return target_id

    def followTarget(self,
                     target_id):

        rate = rospy.Rate(self._rate_control)
        time_pre = rospy.Time.now()

        rospy.loginfo('Began to follow a target with {}.'.format(target_id))
        self._sound_client.say('I have begun to follow a target with {}.'.format(target_id))

        last_target_position_fixedbased = None

        while not rospy.is_shutdown():

            rate.sleep()

            time_current = rospy.Time.now()

            # get cuurent robot position
            try:
                pykdl_transform_fixed_to_robot = tf2_geometry_msgs.transform_to_kdl(
                    self._tf_buffer.lookup_transform(
                        self._frame_fixed,
                        self._frame_robot,
                        rospy.Time(),
                        timeout=rospy.Duration(self._duration_timeout)
                    )
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn('{}'.format(e))
                continue

            #
            if not self._flag_server:
                rospy.logerr('Stop following....')
                self._sound_client.say('Stop following....')
                self._action_client_go_pos.cancel_all_goals()
                self._pub_cmd_vel.publish(Twist())
                return False

            with self._lock_for_dict_objects:
                if target_id not in self._dict_objects:
                    flag_recovery = False
                    if last_target_position_fixedbased is not None:
                        for key in self._dict_objects:
                            if (last_target_position_fixedbased - self._dict_objects[key].position).Norm() < self._threshold_tracking_switching_distance:
                                rospy.loginfo('tracking id switched from {} to {}'.format(target_id,key))
                                self._sound_client.say('tracking id switched from {} to {}'.format(target_id,key))
                                target_id = key
                                flag_recovery = True
                                break

                    if not flag_recovery:
                        rospy.logerr('track id {} is lost..'.format(target_id))
                        self._sound_client.say('I have lost the target with id {}'.format(target_id))
                        self._action_client_go_pos.cancel_all_goals()
                        self._pub_cmd_vel.publish(Twist())
                        return False

                last_target_position_fixedbased = self._dict_objects[target_id].position
                predicted_target_object_position_fixedbased = self._dict_objects[target_id].position + (time_current - time_pre).to_sec() * self._dict_objects[target_id].velocity

            #
            time_pre = time_current
            predicted_target_object_position_robotbased = pykdl_transform_fixed_to_robot.Inverse() * predicted_target_object_position_fixedbased
            #
            target_x_robotbased = predicted_target_object_position_robotbased[0] # meters
            target_y_robotbased = predicted_target_object_position_robotbased[1] # meters
            target_theta_robotbased = math.atan2(target_y_robotbased,target_x_robotbased) # radians
            #
            target_frame_robotbased = PyKDL.Frame(
                                        PyKDL.Rotation.RotZ(target_theta_robotbased),
                                        PyKDL.Vector(
                                                target_x_robotbased,
                                                target_y_robotbased,
                                                0)
                                        )
            destination_frame_targetbased = PyKDL.Frame(
                                        PyKDL.Rotation.RotZ(self._destination_offset_from_target_theta),
                                        PyKDL.Vector(
                                                self._destination_offset_from_target_x,
                                                self._destination_offset_from_target_y,
                                                0)
                                        )
            destination_frame_robotbased = target_frame_robotbased * destination_frame_targetbased
            #destination_frame_robotbased = target_frame_robotbased
            destination_pose_robotbased = PoseStamped()
            destination_pose_robotbased.header.stamp = rospy.Time.now()
            destination_pose_robotbased.header.frame_id = self._frame_robot
            destination_pose_robotbased.pose.position.x = destination_frame_robotbased.p[0]
            destination_pose_robotbased.pose.position.y = destination_frame_robotbased.p[1]
            destination_pose_robotbased.pose.orientation.z = math.sin(target_theta_robotbased/2.0)
            destination_pose_robotbased.pose.orientation.w = math.cos(target_theta_robotbased/2.0)
            # Stop near the target
            if target_frame_robotbased.p.Norm() < 0.5:
                self._action_client_go_pos.cancel_all_goals()
                self._pub_cmd_vel.publish(Twist())
                rospy.loginfo('target is too close {}. robot stops'.format(destination_frame_robotbased.p.Norm()))
                self._pub_destination.publish(destination_pose_robotbased)
                self._sound_client.say('target is too close. robot stops',blocking=True)
                continue
            # Move robot
            goal = TrajectoryGoal()
            goal.duration.data = rospy.Duration(10)
            goal.target_pose = destination_pose_robotbased
            rospy.loginfo('robot moving..')
            if self._use_trajectory:
                ## with trajectory
                self._action_client_go_pos.send_goal(goal)
            else:
                ## with cmd_vel
                ## magick numbers should be rosparam
                max_vx = self._max_vx
                max_vy = self._max_vy
                max_vtheta = self._max_vtheta
                # calc vx, vy, vtheta
                vtheta = destination_theta_robotbased
                vx = vtheta * ( destination_x_robotbased * math.sin( destination_theta_robotbased ) - destination_y_robotbased * math.cos( destination_theta_robotbased ) )
                vy = vtheta * ( destination_x_robotbased * math.cos( destination_theta_robotbased ) + destination_y_robotbased * math.sin( destination_theta_robotbased ) )
                if math.fabs(vx) > max_vx:
                    coef = math.fabs(vx) / max_vx
                    vx *= coef
                    vy *= coef
                    vtheta *= coef
                if math.fabs(vy) > max_vy:
                    coef = math.fabs(vy) / max_vy
                    vx *= coef
                    vy *= coef
                    vtheta *= coef
                if math.fabs(vtheta) > max_vtheta:
                    coef = math.fabs(vtheta) / max_vtheta
                    vx *= coef
                    vy *= coef
                    vtheta *= coef
                # publish cmd_vel
                cmd_vel = Twist()
                cmd_vel.linear.x = vx
                cmd_vel.linear.y = vy
                cmd_vel.angular.z = vtheta
                self._pub_cmd_vel.publish(cmd_vel)
                # publish TwistStamped()
                msg = TwistStamped()
                msg.header.frame_id = self._frame_robot
                msg.header.stamp = rospy.Time.now()
                msg.twist = cmd_vel
                self._pub_twist_command.publish(msg)

            # publish destination
            self._pub_destination.publish(destination_pose_robotbased)

def main():

    rospy.init_node('follow_person_demo')
    demo = Demo()
    while not rospy.is_shutdown():

        rospy.sleep(1)
        with demo._lock_for_flag_server:
            if not demo._flag_server:
                continue
        target_id = demo.getNearestTargetID()
        if target_id is not None:
            demo.followTarget(target_id)
        else:
            rospy.logwarn('None target')
        with demo._lock_for_flag_server:
            demo._flag_server = False


if __name__ == '__main__':
    main()
