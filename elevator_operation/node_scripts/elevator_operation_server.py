#!/usr/bin/env python

import actionlib
import rospy
import roslaunch
import rospkg
from switchbot_ros.switchbot_ros_client import SwitchBotROSClient

import dynamic_reconfigure.client

from std_msgs.msg import Int16
from std_msgs.msg import String
from elevator_operation.msg import DoorState

from elevator_operation.srv import LookAtTarget

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from elevator_operation.msg import MoveFloorWithElevatorAction
from elevator_operation.msg import MoveFloorWithElevatorResult


class ElevatorOperationServer(object):

    def __init__(self):

        #######################################################################
        # ROSLaunch
        #######################################################################
        roslaunch.pmon._init_signal_handlers()
        self.roslaunch_parent = None

        #######################################################################
        # Elevator Config
        #######################################################################
        elevator_config = rospy.get_param('~elevator_config', [])
        self.elevator_config = {entry['floor']: entry for entry in elevator_config}
        rospy.logwarn('elevator config')

        #######################################################################
        # Door Detection Input
        #######################################################################
        self.input_topic_points = rospy.get_param('~input_topic_points')

        #######################################################################
        # ROS Clients
        #######################################################################
        global_costmap_inflation_plugin = rospy.get_param(
                '~global_costmap_inflation_plugin',
                '/move_base/global_costmap/inflation_layer'
                )
        local_costmap_inflation_plugin = rospy.get_param(
                '~local_costmap_inflation_plugin',
                '/move_base/local_costmap/inflation_layer'
                )
        self.client_global_inflater = dynamic_reconfigure.client.Client(global_costmap_inflation_plugin)
        self.client_local_inflater = dynamic_reconfigure.client.Client(local_costmap_inflation_plugin)
        self.switchbot_ros_client = SwitchBotROSClient()
        self.switchbot_ros_client.action_client.wait_for_server()
        self.look_at_client = rospy.ServiceProxy('~look_at', LookAtTarget)
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.logwarn('create ROS client')

        #######################################################################
        # Get Default Inflation Radius
        #######################################################################
        self.update_default_inflation_radius()
        rospy.logwarn('get inflation radius')

        #######################################################################
        # Elevator State and Subscribers
        #######################################################################
        self.state_door_state = DoorState.UNKNOWN
        self.state_elevator_movement = None
        self.state_current_floor = None
        self.subscriber_door_state = rospy.Subscriber(
            '~state_door_state',
            DoorState,
            self.callback_door_state)
        self.subscriber_elevator_movement = rospy.Subscriber(
            '~state_elevator_movement',
            String,
            self.callback_elevator_movement)
        self.subscriber_current_floor = rospy.Subscriber(
            '~state_current_floor',
            Int16,
            self.callback_current_floor)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and \
                (self.state_door_state is None or
                 self.state_elevator_movement is None or
                 self.state_current_floor is None):
            rospy.logwarn('waiting for state messages...')
            rospy.logwarn('door: {}, movement: {}, floor: {}'.format(
                self.state_door_state,
                self.state_elevator_movement,
                self.state_current_floor
                ))
            rate.sleep()

        #######################################################################
        # ROS Action Server
        #######################################################################
        self.action_server = actionlib.SimpleActionServer(
            '~move_floor_with_elevator',
            MoveFloorWithElevatorAction,
            self.execute_cb,
            auto_start=False
        )
        self.action_server.start()

        rospy.loginfo('initialized')

    def callback_door_state(self, msg):
        self.state_door_state = msg.state

    def callback_elevator_movement(self, msg):
        self.state_elevator_movement = msg.data

    def callback_current_floor(self, msg):
        self.state_current_floor = msg.data

    def get_inflation_radius(self):

        cfg_global = self.client_global_inflater.get_configuration()
        cfg_local = self.client_local_inflater.get_configuration()
        return cfg_global['inflation_radius'], cfg_local['inflation_radius']

    def set_inflation_radius(self, global_inflation_radius, local_inflation_radius):

        self.client_global_inflater.update_configuration(
            {'inflation_radius': global_inflation_radius})
        self.client_local_inflater.update_configuration(
            {'inflation_radius': local_inflation_radius})

    def update_default_inflation_radius(self):

        global_inflation_radius, local_inflation_radius = self.get_inflation_radius()
        self.default_global_inflation_radius = global_inflation_radius
        self.default_local_inflation_radius = local_inflation_radius

    def recover_default_inflation_radius(self):

        self.set_inflation_radius(
                self.default_global_inflation_radius,
                self.default_local_inflation_radius
                )

    def start_door_detector(self,
                            input_topic_points,
                            elevator_door_frame_id,
                            door_dimension_x,
                            door_dimension_y,
                            door_dimension_z,
                            door_position_offset='[0,0,0]',
                            door_rotation_offset='[0,0,0]',
                            duration_timeout=10):

        if self.roslaunch_parent is not None:
            self.stop_door_detector()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        roslaunch_path = rospkg.RosPack().get_path('elevator_operation') + '/launch/elevator_door_detector.launch'
        cli_args = [roslaunch_path,
                    'input_topic_points:={}'.format(input_topic_points),
                    'elevator_door_frame_id:={}'.format(elevator_door_frame_id),
                    'door_position_offset:={}'.format(door_position_offset),
                    'door_rotation_offset:={}'.format(door_rotation_offset),
                    'door_dimension_x:={}'.format(door_dimension_x),
                    'door_dimension_y:={}'.format(door_dimension_y),
                    'door_dimension_z:={}'.format(door_dimension_z),
                    ]
        roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
            cli_args[1:]
        )]
        rospy.logwarn('roslaunch_file: {}'.format(roslaunch_file))
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid, roslaunch_file
        )
        self.roslaunch_parent.start()
        try:
            rospy.wait_for_message(
                    '/elevator_door_opening_checker/door_state',
                    DoorState,
                    timeout=duration_timeout)
            rospy.loginfo('Door detector started')
            return True
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('{}'.format(e))
            self.stop_door_detector()
            return False

    def stop_door_detector(self):

        if self.roslaunch_parent is not None:
            self.roslaunch_parent.shutdown()
            self.roslaunch_parent = None

    def _move_to(self, target_pose, wait=False):

        frame_id = target_pose['frame_id']
        position = target_pose['position']
        orientation = target_pose['orientation']

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        self.move_base_client.send_goal(goal)
        if wait:
            self.move_base_client.wait_for_result()

    def execute_cb(self, goal):
        rospy.loginfo('action started')
        result = MoveFloorWithElevatorResult()
        if goal.target_floor_name not in [v['floor_name'] for k, v in self.elevator_config.items()]:
            rospy.logerr('target_floor: {} not in elevator_config'.format(goal.target_floor_name))
            result.success = False
            self.action_server.set_aborted(result)
        else:
            target_floor = filter(
                lambda v: v['floor_name'] == goal.target_floor_name,
                self.elevator_config.values()
            )[0]['floor']
            ret = self.move_floor_with_elevator(target_floor)
            result.success = ret
            self.action_server.set_succeeded(result)
        rospy.loginfo('action finished')

    def move_floor_with_elevator(self, target_floor):

        start_floor = self.state_current_floor

        # move robot to the front of elevator
        self._move_to(
            self.elevator_config[start_floor]['outside_pose'],
            wait=True
        )
        rospy.loginfo('moved to the front of elevator')

        # set inflation_radius
        self.update_default_inflation_radius()
        self.set_inflation_radius(0.2, 0.2)

        # look to the door
        self.look_at_client(self.elevator_config[start_floor]['door_frame_id'])
        rospy.loginfo('look at elevator')

        # Start door detection
        ret = self.start_door_detector(
            self.input_topic_points,
            self.elevator_config[start_floor]['door_frame_id'],
            self.elevator_config[start_floor]['door_dimensions'][0],
            self.elevator_config[start_floor]['door_dimensions'][1],
            self.elevator_config[start_floor]['door_dimensions'][2],
        )
        rospy.loginfo('result of door detector: {}'.format(ret))
        if not ret:
            self.recover_default_inflation_radius()
            return False

        # Get button in current floor and target floor
        if target_floor > start_floor:
            button_from = self.elevator_config[start_floor]['buttons']['up']
            button_to = self.elevator_config[target_floor]['buttons']['down']
        else:
            button_from = self.elevator_config[start_floor]['buttons']['down']
            button_to = self.elevator_config[target_floor]['buttons']['up']

        # Call elevator
        rospy.loginfo('Calling switchbot device: {} action: press'.format(button_from))
        ret = self.switchbot_ros_client.control_device(
            button_from,
            'press',
            wait=True
        )
        rospy.loginfo('Call elevator from start floor: {}'.format(ret))

        # Wait until arrive
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('waiting for door to open.')
            rospy.loginfo('door_state: {}'.format(self.state_door_state))
            if self.state_door_state == DoorState.OPEN:
                break

        # Ride on when arrive
        self._move_to(
            self.elevator_config[start_floor]['inside_pose']
        )
        # Call elevator from target floor
        self.switchbot_ros_client.control_device(
            button_to,
            'press',
            wait=True
        )
        # press current floor button until riding on
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            # press button again
            ret = self.switchbot_ros_client.control_device(
                button_from,
                'press',
                wait=True
            )
            rate.sleep()
            if self.move_base_client.wait_for_result(timeout=rospy.Duration(1)):
                break

        # look to the door
        self.look_at_client(self.elevator_config[start_floor]['door_frame_id'])

        # Wait until arrive
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('door: {}, floor: {}, movement: {}'.format(
                self.state_door_state,
                self.state_current_floor,
                self.state_elevator_movement
                ))
            if self.state_door_state == DoorState.OPEN \
                    and self.state_current_floor == target_floor \
                    and self.state_elevator_movement == 'halt':
                break

        # Get off when arrive
        self._move_to(
            self.elevator_config[target_floor]['outside_pose'],
        )
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            # press button again
            self.switchbot_ros_client.control_device(
                button_to,
                'press',
                wait=True
            )
            rate.sleep()
            if self.move_base_client.wait_for_result(timeout=rospy.Duration(1)):
                break

        # stop door detector
        self.stop_door_detector()

        # recover inflation radius
        self.recover_default_inflation_radius()

        rospy.loginfo('Finished.')
        return True


if __name__ == '__main__':

    rospy.init_node('elevator_operation_server')
    node = ElevatorOperationServer()
    rospy.spin()
