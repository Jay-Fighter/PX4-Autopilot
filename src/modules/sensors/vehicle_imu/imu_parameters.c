/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
* IMU integration rate.
*
* The rate at which raw IMU data is integrated to produce delta angles and delta velocities.
* Recommended to set this to a multiple of the estimator update period (currently 10 ms for ekf2).
*
* @min 100
* @max 1000
* @value 100 100 Hz
* @value 200 200 Hz
* @value 250 250 Hz
* @value 400 400 Hz
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_INT32(IMU_INTEG_RATE, 200);

/**
 * IMU auto calibration
 *
 * Automatically initialize IMU (accel/gyro) calibration from bias estimates if available.
 *
 * @boolean
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_IMU_AUTOCAL, 1);

/**
 * IMU notify clipping
 *
 * Notify the user if the IMU is clipping
 *
 * @boolean
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_IMU_CLPNOTI, 1);

// add by jayjie
/**
 * Accelerometer RPM dynamic notch enable
 *
 * Filter sensor_accel before integration into vehicle_imu (estimator input).
 * Requires mechanical RPM feedback in esc_status. Does not filter the separate
 * vehicle_acceleration output. No FFT is used. Unavailable on constrained-flash builds.
 * Verify estimator performance and CPU load before flight.
 *
 * @boolean
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(IMU_ACC_DNF_EN, 0);

/**
 * Accelerometer RPM dynamic notch bandwidth
 *
 * Independent bandwidth for each motor harmonic. Configuration is loaded at startup.
 *
 * @unit Hz
 * @min 5
 * @max 30
 * @decimal 1
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(IMU_ACC_DNF_BW, 15.f);

/**
 * Accelerometer RPM dynamic notch harmonics
 *
 * Number of mechanical RPM harmonics per ESC and per IMU.
 * Harmonics whose upper half-bandwidth reaches Nyquist are bypassed.
 *
 * @min 1
 * @max 7
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(IMU_ACC_DNF_HMC, 3);

/**
 * Accelerometer RPM dynamic notch minimum frequency
 *
 * At low nonzero RPM, park the fundamental at max(this value, bandwidth).
 * Keep higher harmonics separated by at least half the bandwidth.
 * Zero RPM or telemetry older than 3 seconds bypasses the motor's filters.
 *
 * @unit Hz
 * @min 1
 * @max 1000
 * @decimal 1
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(IMU_ACC_DNF_MIN, 25.f);
// end
