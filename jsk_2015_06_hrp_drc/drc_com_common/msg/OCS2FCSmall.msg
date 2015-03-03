# OCS -> FC
uint8 type
uint8 seq
uint8 image_type
uint8 IMAGE_RECT=0
uint8 PANORAMA_VIEW=1
uint32[8] image_roi_info
float32[32] float_params
# action type
# go-pos
uint8 GO_POS_REGION=1
uint8 GO_POS_EXEC=2
# look-at
uint8 LOOK_AT_REGION=3
#uint8 LOOK_AT_EXEC=4
# valve
uint8 VALVE_REGION=5
uint8 VALVE_EXEC=6
# door
uint8 DOOR_REGION=7
uint8 DOOR_EXEC=8
# debri
uint8 DEBRI_LINE=9
uint8 DEBRI_EXEC=10
# hose_grasp
uint8 HOSE_GRASP_REGION=11
uint8 HOSE_GRASP_EXEC=12
# hose_connect
uint8 HOSE_CONNECT_REGION=13
uint8 HOSE_CONNECT_EXEC=14
# look-around
uint8 LOOK_AROUND_EXEC=15
# drill grasp
uint8 DRILL_GRASP_REGION=16
uint8 DRILL_GRASP_EXEC=17
# drill put
uint8 DRILL_PUT_REGION=18
uint8 DRILL_PUT_EXEC=19
# drill wall
uint8 DRILL_WALL_POINTS=20
uint8 DRILL_WALL_EXEC=21
# drill button
uint8 DRILL_BUTTON_REGION=22
uint8 DRILL_BUTTON_EXEC=23
