/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#include <cmath>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include "MulticopterLandDetector.h"


namespace land_detector
{

MulticopterLandDetector::MulticopterLandDetector() : LandDetector(),
	_paramHandle(),
	_params(),
	_vehicleLocalPositionSub(-1),
	_actuatorsSub(-1),
	_armingSub(-1),
	_attitudeSub(-1),
	_manualSub(-1),
	_ctrl_state_sub(-1),
	_vehicle_control_mode_sub(-1),
	_vehicleLocalPosition{},
	_actuators{},
	_arming{},
	_vehicleAttitude{},
	_manual{},
	_ctrl_state{},
	_ctrl_mode{},
	_min_trust_start(0),
	_arming_time(0)
{
	_paramHandle.maxRotation = param_find("LNDMC_ROT_MAX");  // 20deg/s
	_paramHandle.maxVelocity = param_find("LNDMC_XY_VEL_MAX");  // 1.5 m/s
	_paramHandle.maxClimbRate = param_find("LNDMC_Z_VEL_MAX"); // 0.7m/s
	_paramHandle.maxThrottle = param_find("MPC_THR_MIN"); // 0.12
	_paramHandle.minManThrottle = param_find("MPC_MANTHR_MIN"); // 0.08
	_paramHandle.acc_threshold_m_s2 = param_find("LNDMC_FFALL_THR"); // 2.0
	_paramHandle.ff_trigger_time = param_find("LNDMC_FFALL_TTRI"); // 0.3
}

void MulticopterLandDetector::_initialize_topics()
{
	// subscribe to position, attitude, arming and velocity changes
	// 订阅位置，姿态，解锁以及速度改变
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_actuatorsSub = orb_subscribe(ORB_ID(actuator_controls_0));
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
	_manualSub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
}

void MulticopterLandDetector::_update_topics()
{
	_orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	_orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_orb_update(ORB_ID(actuator_controls_0), _actuatorsSub, &_actuators);
	_orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
	_orb_update(ORB_ID(manual_control_setpoint), _manualSub, &_manual);
	_orb_update(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
	_orb_update(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &_ctrl_mode);
}

void MulticopterLandDetector::_update_params()
{
	param_get(_paramHandle.maxClimbRate, &_params.maxClimbRate);
	param_get(_paramHandle.maxVelocity, &_params.maxVelocity);
	param_get(_paramHandle.maxRotation, &_params.maxRotation_rad_s);
	_params.maxRotation_rad_s = math::radians(_params.maxRotation_rad_s); // 0.349 rad/s
	param_get(_paramHandle.maxThrottle, &_params.maxThrottle);
	param_get(_paramHandle.minManThrottle, &_params.minManThrottle);
	param_get(_paramHandle.acc_threshold_m_s2, &_params.acc_threshold_m_s2);
	param_get(_paramHandle.ff_trigger_time, &_params.ff_trigger_time);
	_freefall_hysteresis.set_hysteresis_time_from(false, 1e6f * _params.ff_trigger_time);// 自由落体滞后时间
}


bool MulticopterLandDetector::_get_freefall_state()
{
	if (_params.acc_threshold_m_s2 < 0.1f
	    || _params.acc_threshold_m_s2 > 10.0f) {	//if parameter is set to zero or invalid, disable free-fall detection. 参数为0或者无效，则禁用自由落体检测
		return false;
	}

	if (_ctrl_state.timestamp == 0) {
		// _ctrl_state is not valid yet, we have to assume we're not falling.
		return false;
	}

	float acc_norm = _ctrl_state.x_acc * _ctrl_state.x_acc
			 + _ctrl_state.y_acc * _ctrl_state.y_acc
			 + _ctrl_state.z_acc * _ctrl_state.z_acc;
	acc_norm = sqrtf(acc_norm);	//norm of specific force. Should be close to 9.8 m/s^2 when landed. 归一化后的力。着陆后加速度应该近似等于9.8m/s^2

	return (acc_norm < _params.acc_threshold_m_s2);	//true if we are currently falling 处于自由落体状态时为真，此时向下的加速度大于8m/s^2
}

bool MulticopterLandDetector::_get_landed_state()
{
	// Time base for this function
	const uint64_t now = hrt_absolute_time();

	float sys_min_throttle = (_params.maxThrottle + 0.01f); // 0.13

	// Determine the system min throttle based on flight mode
	// 基于飞行模式确定系统的最小油门
	if (!_ctrl_mode.flag_control_altitude_enabled) { // 非高度控制模式
		sys_min_throttle = (_params.minManThrottle + 0.01f); // 0.09
	}

	// Check if thrust output is less than the minimum auto throttle param.
	// 检查推力输出是否小于最小自动油门参数
	bool minimalThrust = (_actuators.control[3] <= sys_min_throttle); // 遥控器通道3对应油门

	if (minimalThrust && _min_trust_start == 0) { // 推力输出小于0.13  并且 刚开始达到最小推力
		_min_trust_start = now;

	} else if (!minimalThrust) {
		_min_trust_start = 0;
	}

	// only trigger flight conditions if we are armed
	// 仅当解锁后触发的飞行状态
	if (!_arming.armed) {
		_arming_time = 0;

		return true;

	} else if (_arming_time == 0) {
		_arming_time = now;
	}

	// If in manual flight mode never report landed if the user has more than idle throttle
	// Check if user commands throttle and if so, report not landed based on
	// the user intent to take off (even if the system might physically still have
	// ground contact at this point).
	// 手动模式下，如果用户油门杆量大于idle油门，则不会报告landed。
	if (_manual.timestamp > 0 && _manual.z > 0.15f && _ctrl_mode.flag_control_manual_enabled) { // 使能了手动控制并且油门杆大于0.15
		return false; // 未着陆
	}

	// Return status based on armed state and throttle if no position lock is available.
	// 如果没有位置锁定，则基于解锁状态和油门返回状态
	if (_vehicleLocalPosition.timestamp == 0 ||
	    hrt_elapsed_time(&_vehicleLocalPosition.timestamp) > 500000 ||/* 0.5s */
	    !_vehicleLocalPosition.xy_valid || /* xy方向位置无效 */
	    !_vehicleLocalPosition.z_valid) { /* z方向位置无效 */

		// The system has minimum trust set (manual or in failsafe)
		// if this persists for 8 seconds AND the drone is not
		// falling consider it to be landed. This should even sustain
		// quite acrobatic flight.
		// 系统推力设置最小化
		// 如果持续了8s并且飞机没有下降
		// 这甚至应该保持相当特技的飞行。
		if ((_min_trust_start > 0) &&
		    (hrt_elapsed_time(&_min_trust_start) > 8 * 1000 * 1000)) {

			return true;

		} else {
			return false;
		}
	}

	float armThresholdFactor = 1.0f;

	// Widen acceptance thresholds for landed state right after arming
	// so that motor spool-up and other effects do not trigger false negatives.
	// 解锁后广泛接受油门阈值用于着陆状态
	// 因此电机加速并且其他的影响没有触发错误
	if (hrt_elapsed_time(&_arming_time) < LAND_DETECTOR_ARM_PHASE_TIME_US) { // 2s内解锁
		armThresholdFactor = 2.5f;
	}

	// Check if we are moving vertically - this might see a spike after arming due to
	// throttle-up vibration. If accelerating fast the throttle thresholds will still give
	// an accurate in-air indication.
	// 检查飞机是否在垂直移动 - 由于油门振动，可能会在解锁后看到一个spike。
	// 如果快速加速，油门阈值仍然会给出准确的空中指示。
	bool verticalMovement = fabsf(_vehicleLocalPosition.vz) > _params.maxClimbRate * armThresholdFactor; // 垂直运动 

	// Check if we are moving horizontally.
	// 检查是否存在水平方向上的运动
	bool horizontalMovement = sqrtf(_vehicleLocalPosition.vx * _vehicleLocalPosition.vx
					+ _vehicleLocalPosition.vy * _vehicleLocalPosition.vy) > _params.maxVelocity; // 水平运动

	// Next look if all rotation angles are not moving.
	// 检查是否所有的角都没有运动
	float maxRotationScaled = _params.maxRotation_rad_s * armThresholdFactor; // 角速度

	bool rotating = (fabsf(_vehicleAttitude.rollspeed)  > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.pitchspeed) > maxRotationScaled) ||
			(fabsf(_vehicleAttitude.yawspeed) > maxRotationScaled);


	if (verticalMovement || rotating || !minimalThrust || horizontalMovement) {
		// Sensed movement or thottle high, so reset the land detector.
		// 感受到运动或者油门太高，重置着陆检测器
		return false; // 未着陆
	}

	return true;
}


}
