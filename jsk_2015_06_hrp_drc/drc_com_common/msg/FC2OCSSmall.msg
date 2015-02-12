uint8 type
uint8 seq
uint8 robot_state
# Joint angles of the robot
uint8[34] angles
float32[16] params

# type
# go-pos
uint8 GO_POS_POSE=1
uint8 GO_POS_FINISHED=2
# look-at
uint8 LOOK_AT_POS=3
uint8 LOOK_AT_FINISHED=4

# when type==GO_POS_POSE, 
# params = [x, y, z, yaw, pitch, roll]
# unit is mm.

# robot state
uint8 ROBOT_IDLE=1
uint8 ROBOT_MANIPULATING=2
uint8 ROBOT_GOPOS=3
