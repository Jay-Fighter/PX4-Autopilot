uint64 timestamp # time since system start (microseconds)

uint8 index
float32 throttle_ua  # DC component of throttle
float32 throttle_us  # AC component of throttle
float32 throttle_ctrls_phase # phase angle(rad)
float32 throttle_ctrls_lag_angle # delta angle(rad)
uint8 	frame_enable_flag #0: disable, 1: enable
uint8 	motor_zero_set_flag #0: not set, 1: set
uint8 	encoder_reverse_flag #0: not reverse, 1: reverse
