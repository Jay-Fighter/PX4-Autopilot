uint64 timestamp # time since system start (microseconds)

uint8[4] index
float32[4] throttle_ua  # DC component of throttle
float32[4] throttle_us  # AC component of throttle
float32[4] throttle_ctrls_flap # flap angle(rad)
float32[4] throttle_ctrls_phase # phase angle(rad)
float32[4] throttle_ctrls_lag_angle # delta angle(rad)
uint8 NUM_GROUPS = 4
