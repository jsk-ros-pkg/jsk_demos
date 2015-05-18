# message for vehicle task from OCS to FC

# from operation interface 
float32 handle_cmd # drive/operation/handle_cmd
float32 accel_cmd # drive/operation/handle_cmd

# from vehicle_ui
bool initialize_request
bool grasp_request
bool release_request
bool correct_request
bool resume_request
bool approach_handle_request
bool approach_accel_request
bool reach_arm_request
bool reach_leg_request
