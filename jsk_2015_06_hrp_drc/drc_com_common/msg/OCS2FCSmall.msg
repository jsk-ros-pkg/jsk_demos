# OCS -> FC
uint8 type
uint8 seq
uint32[8] image_roi_info
float32[10] float_params
# action type
# go-pos
uint8 GO_POS_REGION=1
uint8 GO_POS_EXEC=2
# look-at
uint8 LOOK_AT_REGION=3
uint8 LOOK_AT_EXEC=4
# valve
uint8 VALVE_REGION=5
uint8 VALVE_EXEC=6
# door
uint8 DOOR_REGION=7
uint8 DOOR_EXEC=8
# debri
uint8 DEBRI_LINE=9
uint8 DEBRI_EXEC=10
