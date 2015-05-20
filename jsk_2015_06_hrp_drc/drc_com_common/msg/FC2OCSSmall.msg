uint8 type
uint8 seq
# Parameters
float32[32] params
bool success

# type
# go-pos
uint8 REPORT=0
uint8 GO_POS_POSE=1
uint8 GO_POS_FINISHED=2
# look-at
# uint8 LOOK_AT_POS=3
uint8 LOOK_AT_FINISHED=4
# valve
uint8 VALVE_POSE=5
uint8 VALVE_FINISHED=6
# door
uint8 DOOR_POSE=7
uint8 DOOR_FINISHED=8
# debri
uint8 DEBRI_POSE=9
uint8 DEBRI_FINISHED=10
# hose_grasp
uint8 HOSE_GRASP_POSE=11
uint8 HOSE_GRASP_FINISHED=12
# hose_connect
uint8 HOSE_CONNECT_POSE=13
uint8 HOSE_CONNECT_FINISHED=14
# look_around
uint8 LOOK_AROUND_FINISHED=15
# drill_grasp
uint8 DRILL_GRASP_POSE=16
uint8 DRILL_GRASP_FINISHED=17
# drill put
uint8 DRILL_PUT_POSE=18
uint8 DRILL_PUT_FINISHED=19
# drill wall
uint8 DRILL_WALL_POSE=21
uint8 DRILL_WALL_FINISHED=22
# drill button
uint8 DRILL_BUTTON_POSE=23
uint8 DRILL_BUTTON_FINISHED=24
# drill rotate
uint8 DRILL_ROTATE_POSE=25
uint8 DRILL_ROTATE_FINISHED=26
# angle-vector
uint8 ANGLE_VECTOR_FINISHED=27
# for rviz plugins msgs(see src/drc_teleop_interface.cpp)
uint8 RVIZ_MENU_FINISHED=28
# reach until touch
uint8 REACH_UNTIL_TOUCH_FINISHED=29

# failed
uint8 RECOGNITION_FAILED=20

# when type==GO_POS_POSE,
# params = [x, y, z, yaw, pitch, roll]
# unit is mm.
