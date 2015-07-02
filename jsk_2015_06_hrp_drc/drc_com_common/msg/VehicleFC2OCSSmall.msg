# message for vehicle task from FC to OCS

# from driving-controller
# mode -> 0: stop 1: operation 2: recognition
float32 goal_handle_angle # drive/controller/goal_handle_angle
float32 estimated_handle_angle # drive/controller/estimated_handle_angle
float32 step # drive/controller/step
float32 min_step # drive/controller/min_step
float32 max_step # drive/controller/max_step
float32 detach_step # drive/controller/detach_step
float32 neck_y_angle # drive/controller/neck_y_angle
float32 neck_p_angle # drive/controller/neck_p_angle
bool real # drive/controller/real

uint8 handle_mode # drive/controller/handle_mode
uint8 accel_mode # drive/controller/accel_mode
uint8 neck_mode # drive/controller/neck_mode

# to vehicle_ui
bool initialize_response
bool grasp_response
bool release_response
bool correct_response
bool resume_response
bool approach_handle_response
bool approach_accel_response
bool reach_arm_response
bool reach_leg_response
bool egress_response
bool set_max_step_response
bool set_min_step_response
bool set_detach_step_response
bool overwrite_handle_angle_response
bool set_real_response

bool set_handle_mode_response
bool set_accel_mode_response
bool set_neck_mode_response

# from recognition
float32 obstacle_length
