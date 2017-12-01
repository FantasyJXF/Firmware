/*
 * inertial_filter.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include "px4_defines.h"
#include "inertial_filter.h"
#include <cmath>

void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (PX4_ISFINITE(dt)) {
		if (!PX4_ISFINITE(acc)) {
			acc = 0.0f;
		}

		//  距离计算公式
		// s = v * t + 0.5 * a * t^2
		x[0] += x[1] * dt + acc * dt * dt / 2.0f; 
		// 速度计算公式
		// v = a * t
		x[1] += acc * dt;
	}
}

void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (PX4_ISFINITE(e) && PX4_ISFINITE(w) && PX4_ISFINITE(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}
