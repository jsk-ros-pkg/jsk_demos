uint8 type
uint8 seq
uint8 robot_state
# Parameters
float32[16] params

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
# drill_put
uint8 DRILL_PUT_POSE=17
uint8 DRILL_PUT_FINISHED=18

# failed
uint8 RECOGNITION_FAILED=19

# when type==GO_POS_POSE, 
# params = [x, y, z, yaw, pitch, roll]
# unit is mm.

# robot state
uint8 ROBOT_IDLE=1
uint8 ROBOT_MANIPULATING=2
uint8 ROBOT_GOPOS=3
