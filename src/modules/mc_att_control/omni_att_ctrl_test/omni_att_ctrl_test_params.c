/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// add by jayjie
/**
 * Enable omni attitude controller test setpoint publishing
 *
 * When enabled, omni_att_ctrl_test publishes vehicle_attitude_setpoint using OMNI_ATT_ROLL,
 * OMNI_ATT_PITCH, OMNI_ATT_YAW, and OMNI_ATT_THR_Z.
 *
 * @boolean
 * @group Omni Attitude Test
 */
PARAM_DEFINE_INT32(OMNI_ATT_EN, 1);

/**
 * Omni attitude test roll setpoint
 *
 * Desired roll attitude published by omni_att_ctrl_test.
 *
 * @unit deg
 * @min -45
 * @max 45
 * @decimal 2
 * @increment 0.5
 * @group Omni Attitude Test
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_ROLL, 0.0f);

/**
 * Omni attitude test pitch setpoint
 *
 * Desired pitch attitude published by omni_att_ctrl_test.
 *
 * @unit deg
 * @min -45
 * @max 45
 * @decimal 2
 * @increment 0.5
 * @group Omni Attitude Test
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_PITCH, 0.0f);

/**
 * Omni attitude test yaw setpoint
 *
 * Desired yaw attitude published by omni_att_ctrl_test.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 2
 * @increment 1
 * @group Omni Attitude Test
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_YAW, 0.0f);

/**
 * Omni attitude test body Z thrust setpoint
 *
 * Normalized multicopter body-FRD Z thrust. Upward thrust is negative.
 *
 * @min -1
 * @max 0
 * @decimal 3
 * @increment 0.01
 * @group Omni Attitude Test
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_THR_Z, -0.5f);

/**
 * Omni attitude test publish rate
 *
 * Publish rate for vehicle_attitude_setpoint.
 *
 * @unit Hz
 * @min 1
 * @max 200
 * @decimal 1
 * @increment 10
 * @group Omni Attitude Test
 */
PARAM_DEFINE_FLOAT(OMNI_ATT_RATE, 100.0f);
// end
