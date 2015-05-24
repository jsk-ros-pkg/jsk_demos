float32[35] angles
uint8[35] effort
# 0~255 degree
uint8[35] temperature
# lfx, lfy, lfz, lfrx, lfry, lfrz, lfx, rfy, rfz, rfrx, rfry, rfrz
int8[12] arm_forces
int8[12] leg_forces
float32[6] odom_on_ground
float32[6] ground
float32[6] odom
float32[4] imu                  #[x, y, z, w]
uint8 robot_state
