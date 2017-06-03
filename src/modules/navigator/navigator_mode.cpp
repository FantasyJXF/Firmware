/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file navigator_mode.cpp
 *
 * Base class for different modes in navigator
 * navigator模块中不同模式的基类
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "navigator_mode.h"
#include "navigator.h"

NavigatorMode::NavigatorMode(Navigator *navigator, const char *name) :
	SuperBlock(navigator, name),
	_navigator(navigator),
	_first_run(true)
{
	/* load initial params */
	updateParams();
	/* set initial mission items */
	on_inactive();
}

NavigatorMode::~NavigatorMode()
{
}

void
NavigatorMode::run(bool active)
{
	if (active) {
		if (_first_run) {
			/* first run */
			_first_run = false;
			/* Reset stay in failsafe flag */
			_navigator->get_mission_result()->stay_in_failsafe = false;  // 退出failsafe模式
			_navigator->set_mission_result_updated();
			on_activation();

		} else {
			/* periodic updates when active */
			// 激活时定期更新
			on_active();
		}

	} else {
		/* periodic updates when inactive */
		// 未激活时定期更新
		_first_run = true;
		on_inactive();
	}
}

void
NavigatorMode::on_inactive()
{
}

void
NavigatorMode::on_activation()
{
	/* invalidate position setpoint by default */
	// 默认情况下位置设定值无效
	_navigator->get_position_setpoint_triplet()->current.valid = false;
}

void
NavigatorMode::on_active()
{
}
