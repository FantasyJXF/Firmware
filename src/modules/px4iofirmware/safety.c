/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file safety.c
 * Safety button logic.
 */

#include <px4_config.h>

#include <stdbool.h>

#include <drivers/drv_hrt.h>

#include "px4io.h"

static struct hrt_call arming_call;
static struct hrt_call failsafe_call;

/*
 * Count the number of times in a row that we see the arming button
 * held down.
 * 计数，解锁安全开关按下
 */
static unsigned counter = 0;

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking	慢闪	*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 		0x5555		/**< fast blinking	快闪	*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 双闪	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 	四闪	*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on	 常亮 */

static unsigned blink_counter = 0;

/*
 * IMPORTANT: The arming state machine critically
 * 	     	depends on using the same threshold
 *            for arming and disarming. Since disarming
 *            is quite deadly for the system, a similar
 *            length can be justified.
 * 			解锁状态机严重依赖使用相同的阈值用于解锁和上锁
 *            因为上锁对于系统是非常致命的，类似的长度可以说是合理的。
 */
#define ARM_COUNTER_THRESHOLD	10

static bool safety_button_pressed;

static void safety_check_button(void *arg);
static void failsafe_blink(void *arg);

void
safety_init(void)
{
	/* arrange for the button handler to be called at 10Hz */
	// 按钮检查频率10Hz
	hrt_call_every(&arming_call, 1000, 100000, safety_check_button, NULL);
}

void
failsafe_led_init(void)
{
	/* arrange for the failsafe blinker to be called at 8Hz */
	hrt_call_every(&failsafe_call, 1000, 125000, failsafe_blink, NULL);
}

static void
safety_check_button(void *arg)
{
	/*
	 * Debounce the safety button, change state if it has been held for long enough.
	 * 按下安全开关一段时间后弹回，如果已经按了足够长时间则改变状态
	 *
	 */
	safety_button_pressed = BUTTON_SAFETY; // 读取安全开关

	/*
	 * Keep pressed for a while to arm.
	 *
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
	 */
	if (safety_button_pressed && !(r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) /* 当前为IO锁定状态时，此项为真 */&&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK)) { /* 锁定 -> 解锁 */

		if (counter < ARM_COUNTER_THRESHOLD) {
			counter++;

		} else if (counter == ARM_COUNTER_THRESHOLD) {
			/* switch to armed state */
			// 切换到解锁状态
			r_status_flags |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF; // 按位或，关闭安全保护
			counter++;
		}

	} else if (safety_button_pressed && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)) { /* 解锁到锁定 */

		if (counter < ARM_COUNTER_THRESHOLD) {
			counter++;

		} else if (counter == ARM_COUNTER_THRESHOLD) {
			/* change to disarmed state and notify the FMU */
			// 切换到上锁状态并通知FMU
			r_status_flags &= ~PX4IO_P_STATUS_FLAGS_SAFETY_OFF; // 安全状态位置0 
			counter++;
		}

	} else {
		counter = 0;
	}

	/* Select the appropriate LED flash pattern depending on the current IO/FMU arm state */
	// 根据当前IO/FMU的解锁状态选取恰当的LED闪烁节奏
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) { // 当前IO保护关闭
		if (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED) {  // FMU已解锁
			pattern = LED_PATTERN_IO_FMU_ARMED; // 常亮

		} else { // FMU未解锁(pixhawk的FMU就是为解锁状态，安全开关灯双闪表示关闭了安全保护)
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED) { // IO安全保护打开，FMU已解锁
		pattern = LED_PATTERN_FMU_ARMED;

	} else if (r_setup_arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK) { // 同意IO解锁
		pattern = LED_PATTERN_FMU_OK_TO_ARM; // 慢闪(orearmed状态下安全开关灯状态)

	}

	/* Turn the LED on if we have a 1 at the current bit position */
	LED_SAFETY(pattern & (1 << blink_counter++)); // 安全开关状态灯

	if (blink_counter > 15) {
		blink_counter = 0;
	}
}

static void
failsafe_blink(void *arg)
{
	/* indicate that a serious initialisation error occured */
	if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
		LED_AMBER(true);
		return;
	}

	static bool failsafe = false;

	/* blink the failsafe LED if we don't have FMU input */
	if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {
		failsafe = !failsafe;

	} else {
		failsafe = false;
	}

	LED_AMBER(failsafe);
}
