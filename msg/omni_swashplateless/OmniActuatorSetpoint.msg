# Omni control message


uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled


uint8 NUM_CONTROLS = 12
float32[12] control # range: [-1, 1], where 1 means maximum positive thrust,
                    # -1 maximum negative (if not supported by the output, <0 maps to NaN),
                    # and NaN maps to disarmed (stop the motors)
float32[4] fx
float32[4] fy
float32[4] fz
float32[4] f # total thrust

float32 fx_body
float32 fy_body
float32 fz_body
uint8 num_groups
