#!/bin/sh

PREFIX=$1

exec rosbag record --split --size=2048 -o "$PREFIX" /joint_states_throttle /rasensor /lasensor /rfsensor /lfsensor /off_rasensor /off_lasensor /off_rfsensor /off_lfsensor /motor_states/cur_angle /motor_states/motor_current /motor_states/servo_alarm /motor_states_low/motor_outer_temp /motor_states_low/motor_temp /motor_states/ref_angle /imu /tf /motor_states_low/abs_cur_angle_diff /ref_lasensor /ref_rasensor /ref_lfsensor /ref_rfsensor /motor_states_low/board_vin /staro_drive/operation/flag/handle /staro_drive/operation/flag/accel /staro_drive/operation/handle_cmd /staro_drive/operation/accel_cmd /staro_drive/operation/brake_cmd
