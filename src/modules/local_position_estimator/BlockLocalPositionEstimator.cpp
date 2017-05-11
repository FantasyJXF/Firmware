#include "BlockLocalPositionEstimator.hpp"
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>
#include <cstdlib>

orb_advert_t mavlink_log_pub = nullptr;

// timeouts for sensors in microseconds
static const uint32_t 		EST_SRC_TIMEOUT = 10000; // 0.01 s

// required standard deviation of estimate for estimator to publish data
static const uint32_t 		EST_STDDEV_XY_VALID = 2.0; // 2.0 m
static const uint32_t 		EST_STDDEV_Z_VALID = 2.0; // 2.0 m
static const uint32_t 		EST_STDDEV_TZ_VALID = 2.0; // 2.0 m
static const bool integrate = true; // use accel for integrating

static const float P_MAX = 1.0e6f; // max allowed value in state covariance
static const float LAND_RATE = 10.0f; // rate of land detector correction 着陆检测器校正速率

BlockLocalPositionEstimator::BlockLocalPositionEstimator() : // 一般构造函数
	// this block has no parent, and has name LPE
	SuperBlock(NULL, "LPE"),
	// subscriptions, set rate, add to list
	_sub_armed(ORB_ID(actuator_armed), 1000 / 2, 0, &getSubscriptions()),
	_sub_land(ORB_ID(vehicle_land_detected), 1000 / 2, 0, &getSubscriptions()),
	_sub_att(ORB_ID(vehicle_attitude), 1000 / 100, 0, &getSubscriptions()),
	// set flow max update rate higher than expected to we don't lose packets
	// 设置光流的最大更新速率高于期望值，避免丢包
	_sub_flow(ORB_ID(optical_flow), 1000 / 100, 0, &getSubscriptions()),
	// main prediction loop, 100 hz
	// 主预测环 100Hz
	_sub_sensor(ORB_ID(sensor_combined), 1000 / 100, 0, &getSubscriptions()),
	// status updates 2 hz
	// 状态更新2Hz
	_sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),
	_sub_manual(ORB_ID(manual_control_setpoint), 1000 / 2, 0, &getSubscriptions()),
	// gps 10 hz
	_sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),
	// vision 30 hz
	_sub_vision_pos(ORB_ID(vision_position_estimate), 1000 / 30, 0, &getSubscriptions()),
	// mocap 50 hz
	_sub_mocap(ORB_ID(att_pos_mocap), 1000 / 50, 0, &getSubscriptions()),
	// all distance sensors, 10 hz
	_sub_dist0(ORB_ID(distance_sensor), 1000 / 10, 0, &getSubscriptions()),
	_sub_dist1(ORB_ID(distance_sensor), 1000 / 10, 1, &getSubscriptions()),
	_sub_dist2(ORB_ID(distance_sensor), 1000 / 10, 2, &getSubscriptions()),
	_sub_dist3(ORB_ID(distance_sensor), 1000 / 10, 3, &getSubscriptions()),
	_dist_subs(),
	_sub_lidar(NULL),
	_sub_sonar(NULL),

	// publications
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),
	_pub_innov(ORB_ID(ekf2_innovations), -1, &getPublications()),

	// map projection
	_map_ref(),

	// block parameters
	_pub_agl_z(this, "PUB_AGL_Z"),
	_vxy_pub_thresh(this, "VXY_PUB"),
	_z_pub_thresh(this, "Z_PUB"),
	_sonar_z_stddev(this, "SNR_Z"),
	_sonar_z_offset(this, "SNR_OFF_Z"),
	_lidar_z_stddev(this, "LDR_Z"),
	_lidar_z_offset(this, "LDR_OFF_Z"),
	_accel_xy_stddev(this, "ACC_XY"),
	_accel_z_stddev(this, "ACC_Z"),
	_baro_stddev(this, "BAR_Z"),
	_gps_on(this, "GPS_ON"),
	_gps_delay(this, "GPS_DELAY"),
	_gps_xy_stddev(this, "GPS_XY"),
	_gps_z_stddev(this, "GPS_Z"),
	_gps_vxy_stddev(this, "GPS_VXY"),
	_gps_vz_stddev(this, "GPS_VZ"),
	_gps_eph_max(this, "EPH_MAX"),
	_gps_epv_max(this, "EPV_MAX"),
	_vision_xy_stddev(this, "VIS_XY"),
	_vision_z_stddev(this, "VIS_Z"),
	_vision_delay(this, "VIS_DELAY"),
	_vision_on(this, "VIS_ON"),
	_mocap_p_stddev(this, "VIC_P"),
	_flow_gyro_comp(this, "FLW_GYRO_CMP"),
	_flow_z_offset(this, "FLW_OFF_Z"),
	_flow_scale(this, "FLW_SCALE"),
	//_flow_board_x_offs(NULL, "SENS_FLW_XOFF"),
	//_flow_board_y_offs(NULL, "SENS_FLW_YOFF"),
	_flow_min_q(this, "FLW_QMIN"),
	_land_z_stddev(this, "LAND_Z"),
	_pn_p_noise_density(this, "PN_P"),
	_pn_v_noise_density(this, "PN_V"),
	_pn_b_noise_density(this, "PN_B"),
	_pn_t_noise_density(this, "PN_T"),
	_t_max_grade(this, "T_MAX_GRADE"),

	// init origin
	_init_origin_lat(this, "LAT"),
	_init_origin_lon(this, "LON"),

	// flow gyro
	_flow_gyro_x_high_pass(this, "FGYRO_HP"),
	_flow_gyro_y_high_pass(this, "FGYRO_HP"),

	// stats
	_baroStats(this, ""),
	_sonarStats(this, ""),
	_lidarStats(this, ""),
	_flowQStats(this, ""),
	_visionStats(this, ""),
	_mocapStats(this, ""),
	_gpsStats(this, ""),

	// low pass
	_xLowPass(this, "X_LP"), // 状态发布的截止频率
	// use same lp constant for agl
	_aglLowPass(this, "X_LP"),

	// delay
	_xDelay(this, ""),
	_tDelay(this, ""),

	// misc
	_polls(),
	_timeStamp(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_xy(0),
	_time_last_z(0),
	_time_last_tz(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_init_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),
	_time_last_land(0),

	// initialization flags
	_receivedGps(false),
	_baroInitialized(false),
	_gpsInitialized(false),
	_lidarInitialized(false),
	_sonarInitialized(false),
	_flowInitialized(false),
	_visionInitialized(false),
	_mocapInitialized(false),
	_landInitialized(false),

	// reference altitudes
	_altOrigin(0),
	_altOriginInitialized(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	// status
	_validXY(false),
	_validZ(false),
	_validTZ(false),
	_xyTimeout(true),
	_zTimeout(true),
	_tzTimeout(true),
	_lastArmedState(false),

	// faults
	_baroFault(FAULT_NONE),
	_gpsFault(FAULT_NONE),
	_lidarFault(FAULT_NONE),
	_flowFault(FAULT_NONE),
	_sonarFault(FAULT_NONE),
	_visionFault(FAULT_NONE),
	_mocapFault(FAULT_NONE),
	_landFault(FAULT_NONE),

	// loop performance
	_loop_perf(),
	_interval_perf(),
	_err_perf(),

	// kf matrices
	// 卡尔曼滤波器的几个矩阵
	_x(), _u(), _P(), _R_att(), _eul()
{
	// assign distance subs to array
	_dist_subs[0] = &_sub_dist0;
	_dist_subs[1] = &_sub_dist1;
	_dist_subs[2] = &_sub_dist2;
	_dist_subs[3] = &_sub_dist3;

	// setup event triggering based on new flow messages to integrate
	// 基于要积分的新的光流消息设置事件触发
	_polls[POLL_FLOW].fd = _sub_flow.getHandle();   // fd[0] 为感兴趣的光流数据
	_polls[POLL_FLOW].events = POLLIN;

	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();  // fd[1]为感兴趣的参数更新
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();   // fd[2]为感兴趣的传感器数据
	_polls[POLL_SENSORS].events = POLLIN;

	// initialize A, B,  P, x, u
////////////////  初始化	
	_x.setZero();
	_u.setZero();
	initSS();

	// perf counters
	_loop_perf = perf_alloc(PC_ELAPSED,
				"local_position_estimator_runtime");
	//_interval_perf = perf_alloc(PC_INTERVAL,
	//"local_position_estimator_interval");
	_err_perf = perf_alloc(PC_COUNT, "local_position_estimator_err");

	// map
	_map_ref.init_done = false;

	// intialize parameter dependent matrices
	// 初始化参数依赖矩阵
	updateParams();
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator()
{
}

Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return _A * x + _B * u;
}

void BlockLocalPositionEstimator::update()
{

	// wait for a sensor update, check for exit condition every 100 ms
	// 等待传感器更新
	int ret = px4_poll(_polls, 3, 100);

	if (ret < 0) {
		/* poll error, count it in perf */
		perf_count(_err_perf);
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

	// auto-detect connected rangefinders while not armed
	// 未解锁时自动检测已连接的测距仪
	bool armedState = _sub_armed.get().armed;

	if (!armedState && (_sub_lidar == NULL || _sub_sonar == NULL)) {
		detectDistanceSensors(); // 检测是什么测距仪 并 更新距离
	}

	// reset pos, vel, and terrain on arming
	// 解锁时重置位置、速度、海拔高度

	// XXX this will be re-enabled for indoor use cases using a
	// selection param, but is really not helping outdoors
	// right now.
	// 下列被注释的内容将使用参数选择重新用于室内飞行

	// if (!_lastArmedState && armedState) {

	// 	// we just armed, we are at origin on the ground
	// 	_x(X_x) = 0;
	// 	_x(X_y) = 0;
	// 	// reset Z or not? _x(X_z) = 0;

	// 	// we aren't moving, all velocities are zero
	// 	_x(X_vx) = 0;
	// 	_x(X_vy) = 0;
	// 	_x(X_vz) = 0;

	// 	// assume we are on the ground, so terrain alt is local alt
	// 	_x(X_tz) = _x(X_z);

	// 	// reset lowpass filter as well
	// 	_xLowPass.setState(_x);
	// 	_aglLowPass.setState(0);
	// }

	_lastArmedState = armedState;

	// see which updates are available
	// 检查可用更新
	bool flowUpdated = _sub_flow.updated();
	bool paramsUpdated = _sub_param_update.updated();
	bool baroUpdated = _sub_sensor.updated(); // sensor_combined
	bool gpsUpdated = _gps_on.get() && _sub_gps.updated();
	bool visionUpdated = _vision_on.get() && _sub_vision_pos.updated();
	bool mocapUpdated = _sub_mocap.updated();
	bool lidarUpdated = (_sub_lidar != NULL) && _sub_lidar->updated();
	bool sonarUpdated = (_sub_sonar != NULL) && _sub_sonar->updated();
	bool landUpdated = (
				   (_sub_land.get().landed ||
				    ((!_sub_armed.get().armed) && (!_sub_land.get().freefall)))
				   && (!(_lidarInitialized || _mocapInitialized || _visionInitialized || _sonarInitialized))
				   && ((_timeStamp - _time_last_land) > 1.0e6f / LAND_RATE));

	// get new data
	// 获取新数据
	updateSubscriptions();

	// update parameters
	// 更新参数
	if (paramsUpdated) {
		updateParams();
		updateSSParams(); // 更新噪声矩阵
	}

	// is xy valid?
	bool vxy_stddev_ok = false;

	if (math::max(_P(X_vx, X_vx), _P(X_vy, X_vy)) < _vxy_pub_thresh.get()*_vxy_pub_thresh.get()) {
		vxy_stddev_ok = true;
	}

	if (_validXY) {
		// if valid and gps has timed out, set to not valid
		// 如果其有效，而且GPS超时了，那么还是设置其无效
		if (!vxy_stddev_ok && !_gpsInitialized) {
			_validXY = false;
		}

	} else {
		if (vxy_stddev_ok) {
			if (_flowInitialized || _gpsInitialized || _visionInitialized || _mocapInitialized) {
				_validXY = true; // 各种设备初始化完成，设置 XY位置有效
			}
		}
	}

	// is z valid?
	bool z_stddev_ok = sqrtf(_P(X_z, X_z)) < _z_pub_thresh.get(); // 标准差在阈值内

	if (_validZ) {
		// if valid and baro has timed out, set to not valid
		// Z有效，但是气压计超时，设置为无效
		if (!z_stddev_ok && !_baroInitialized) {
			_validZ = false;
		}

	} else {
		if (z_stddev_ok) {
			_validZ = true;
		}
	}

	// is terrain valid?
	bool tz_stddev_ok = sqrtf(_P(X_tz, X_tz)) < _z_pub_thresh.get();

	if (_validTZ) {
		if (!tz_stddev_ok) {
			_validTZ = false;
		}

	} else {
		if (tz_stddev_ok) {
			_validTZ = true;
		}
	}

	// timeouts
	if (_validXY) {
		_time_last_xy = _timeStamp;
	}

	if (_validZ) {
		_time_last_z = _timeStamp;
	}

	if (_validTZ) {
		_time_last_tz = _timeStamp;
	}

	// check timeouts
	// 检测位置估计以及各种传感器是否超时
	checkTimeouts();

	// if we have no lat, lon initialize projection at 0,0
	// GPS无效，初始化原点投影在 0,0
	if (_validXY && !_map_ref.init_done) {
		map_projection_init(&_map_ref,
				    _init_origin_lat.get(),
				    _init_origin_lon.get());
	}

	// reinitialize x if necessary
	// 必要时重新初始化x
	bool reinit_x = false;

	for (int i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		// 是否应该在这里重新初始化传感器
		if (!PX4_ISFINITE(_x(i))) {
			reinit_x = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] reinit x, x(%d) not finite", i);
			break;
		}
	}

	if (reinit_x) {
		for (int i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] reinit x");
	}

	// force P symmetry and reinitialize P if necessary
	// 强制P对称，必要时重新初始化P
	bool reinit_P = false;

	for (int i = 0; i < n_x; i++) {
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				mavlink_and_console_log_info(&mavlink_log_pub,
							     "[lpe] reinit P (%d, %d) not finite", i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				// 确保对角线元素为正
				if (_P(i, i) <= 0) {
					mavlink_and_console_log_info(&mavlink_log_pub,
								     "[lpe] reinit P (%d, %d) negative", i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force symmetry
				// 从上三角形复制元素以强制对称
				_P(j, i) = _P(i, j);
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		initP();
	}

	// do prediction
	// EKF的预测阶段
	predict(); // 模型预测

	// sensor corrections/ initializations
////////////// 传感器校正/初始化///////////
	if (gpsUpdated) {
		if (!_gpsInitialized) {
			gpsInit(); // GPS初始化/测量

		} else {
			gpsCorrect(); // GPS测量/校正   EKF的校正阶段
		}
	}

	if (baroUpdated) {
		if (!_baroInitialized) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (lidarUpdated) {
		if (!_lidarInitialized) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (sonarUpdated) {
		if (!_sonarInitialized) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (flowUpdated) {
		if (!_flowInitialized) {
			flowInit();

		} else {
			perf_begin(_loop_perf);// TODO
			flowCorrect();
			//perf_count(_interval_perf);
			perf_end(_loop_perf);
		}
	}

	if (visionUpdated) {
		if (!_visionInitialized) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (mocapUpdated) {
		if (!_mocapInitialized) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (landUpdated) {
		if (!_landInitialized) {
			landInit();

		} else {
			landCorrect();
		}
	}

	if (_altOriginInitialized) {
		// update all publications if possible
		// 更新所有的发布
		publishLocalPos(); // 发布位置
		publishEstimatorStatus(); // 发布估计状态
		_pub_innov.update(); // 发布EKF2的新息 ??

		if (_validXY) {
			publishGlobalPos(); // 发布全球位置信息
		}
	}

	// propagate delayed state, no matter what
	// if state is frozen, delayed state still
	// needs to be propagated with frozen state
	float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);

	if (_time_last_hist == 0 ||
	    (dt_hist > HIST_STEP)) {
		_tDelay.update(Scalar<uint64_t>(_timeStamp)); // 更新滞后时间
		_xDelay.update(_x);
		_time_last_hist = _timeStamp;
	}
}

void BlockLocalPositionEstimator::checkTimeouts()
{
	if (_timeStamp - _time_last_xy > EST_SRC_TIMEOUT) {
		if (!_xyTimeout) {
			_xyTimeout = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] xy timeout ");
		}

	} else if (_xyTimeout) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] xy resume ");
		_xyTimeout = false;
	}

	if (_timeStamp - _time_last_z > EST_SRC_TIMEOUT) {
		if (!_zTimeout) {
			_zTimeout = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] z timeout ");
		}

	} else if (_zTimeout) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] z resume ");
		_zTimeout = false;
	}

	if (_timeStamp - _time_last_tz > EST_SRC_TIMEOUT) {
		if (!_tzTimeout) {
			_tzTimeout = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] tz timeout ");
		}

	} else if (_tzTimeout) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] tz resume ");
		_tzTimeout = false;
	}

	// 各种设备的超时阈值不同，因此分开进行检测
	lidarCheckTimeout();
	sonarCheckTimeout();
	baroCheckTimeout();
	gpsCheckTimeout();
	flowCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
}

float BlockLocalPositionEstimator::agl()
{
	return _x(X_tz) - _x(X_z);
}

void BlockLocalPositionEstimator::correctionLogic(Vector<float, n_x> &dx)
{
	// don't correct bias when rotating rapidly
	float ang_speed = sqrtf( // 计算角速度 sqrt(phi^2 + theta^2 + psi^2)
				  _sub_att.get().rollspeed * _sub_att.get().rollspeed +
				  _sub_att.get().pitchspeed * _sub_att.get().pitchspeed +
				  _sub_att.get().yawspeed * _sub_att.get().yawspeed);

	if (ang_speed > 1) {
		dx(X_bx) = 0;
		dx(X_by) = 0;
		dx(X_bz) = 0;
	}

	// if xy not valid, stop estimating
	// 水平位置无效，停止估计
	if (!_validXY) {
		dx(X_x) = 0;
		dx(X_y) = 0;
		dx(X_vx) = 0;
		dx(X_vy) = 0;
		dx(X_bx) = 0;
		dx(X_by) = 0;
	}

	// if z not valid, stop estimating
	// 垂直位置无效，停止估计
	if (!_validZ) {
		dx(X_z) = 0;
		dx(X_vz) = 0;
		dx(X_bz) = 0;
	}

	// if terrain not valid, stop estimating
	// 地面海拔高度无效，停止估计
	if (!_validTZ) {
		dx(X_tz) = 0;
	}

	// saturate bias
	// 累加加速度偏差 供后面检测
	float bx = dx(X_bx) + _x(X_bx); // _x状态变量中的加速度偏差始终在更新
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) {
		bx = BIAS_MAX * bx / std::abs(bx); // bx = 0.1 * bx
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}
}


void BlockLocalPositionEstimator::covPropagationLogic(Matrix<float, n_x, n_x> &dP)
{
	for (int i = 0; i < n_x; i++) {
		if (_P(i, i) > P_MAX) {
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (int j = 0; j < n_x; j++) {
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}
}

void BlockLocalPositionEstimator::detectDistanceSensors()
{
	for (int i = 0; i < N_DIST_SUBS; i++) {
		uORB::Subscription<distance_sensor_s> *s = _dist_subs[i];

		if (s == _sub_lidar || s == _sub_sonar) { continue; }

		if (s->updated()) { // 发布是否已更新
			s->update(); // 更新距离

			if (s->get().timestamp == 0) { continue; }

			if (s->get().type == \
			    distance_sensor_s::MAV_DISTANCE_SENSOR_LASER && /* 激光测距 */
			    _sub_lidar == NULL) {
				_sub_lidar = s;
				mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] Lidar detected with ID %i", i);

			} else if (s->get().type == \
				   distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND && /* 超声波测距 */
				   _sub_sonar == NULL) {
				_sub_sonar = s;
				mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] Sonar detected with ID %i", i);
			}
		}
	}
}


void BlockLocalPositionEstimator::publishLocalPos()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float vxy_stddev = sqrtf(_P(X_vx, X_vx) + _P(X_vy, X_vy));
	float epv = sqrtf(_P(X_z, X_z));
	float eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (vxy_stddev < _vxy_pub_thresh.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	// publish local position
	if (PX4_ISFINITE(_x(X_x)) && PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {
		_pub_lpos.get().timestamp = _timeStamp;
		_pub_lpos.get().xy_valid = _validXY;
		_pub_lpos.get().z_valid = _validZ;
		_pub_lpos.get().v_xy_valid = _validXY;
		_pub_lpos.get().v_z_valid = _validZ;
		_pub_lpos.get().x = xLP(X_x); 	// north
		_pub_lpos.get().y = xLP(X_y);  	// east

		if (_pub_agl_z.get()) {
			_pub_lpos.get().z = -_aglLowPass.getState(); // agl

		} else {
			_pub_lpos.get().z = xLP(X_z); 	// down
		}

		_pub_lpos.get().vx = xLP(X_vx); // north
		_pub_lpos.get().vy = xLP(X_vy); // east
		_pub_lpos.get().vz = xLP(X_vz); // down

		_pub_lpos.get().yaw = _eul(2);
		_pub_lpos.get().xy_global = _validXY;
		_pub_lpos.get().z_global = _baroInitialized;
		_pub_lpos.get().ref_timestamp = _timeStamp;
		_pub_lpos.get().ref_lat = _map_ref.lat_rad * 180 / M_PI;
		_pub_lpos.get().ref_lon = _map_ref.lon_rad * 180 / M_PI;
		_pub_lpos.get().ref_alt = _altOrigin;
		_pub_lpos.get().dist_bottom = _aglLowPass.getState();
		_pub_lpos.get().dist_bottom_rate = - xLP(X_vz);
		_pub_lpos.get().surface_bottom_timestamp = _timeStamp;
		// we estimate agl even when we don't have terrain info
		// if you are in terrain following mode this is important
		// so that if terrain estimation fails there isn't a
		// sudden altitude jump
		_pub_lpos.get().dist_bottom_valid = _validZ;
		_pub_lpos.get().eph = eph;
		_pub_lpos.get().epv = epv;
		_pub_lpos.update();
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	_pub_est_status.get().timestamp = _timeStamp;

	for (int i = 0; i < n_x; i++) {
		_pub_est_status.get().states[i] = _x(i);
		_pub_est_status.get().covariances[i] = _P(i, i);
	}

	_pub_est_status.get().n_states = n_x;
	_pub_est_status.get().nan_flags = 0;
	_pub_est_status.get().health_flags =
		((_baroFault > FAULT_NONE) << SENSOR_BARO)
		+ ((_gpsFault > FAULT_NONE) << SENSOR_GPS)
		+ ((_lidarFault > FAULT_NONE) << SENSOR_LIDAR)
		+ ((_flowFault > FAULT_NONE) << SENSOR_FLOW)
		+ ((_sonarFault > FAULT_NONE) << SENSOR_SONAR)
		+ ((_visionFault > FAULT_NONE) << SENSOR_VISION)
		+ ((_mocapFault > FAULT_NONE) << SENSOR_MOCAP);
	_pub_est_status.get().timeout_flags =
		(_baroInitialized << SENSOR_BARO)
		+ (_gpsInitialized << SENSOR_GPS)
		+ (_flowInitialized << SENSOR_FLOW)
		+ (_lidarInitialized << SENSOR_LIDAR)
		+ (_sonarInitialized << SENSOR_SONAR)
		+ (_visionInitialized << SENSOR_VISION)
		+ (_mocapInitialized << SENSOR_MOCAP);
	_pub_est_status.get().pos_horiz_accuracy = _pub_gpos.get().eph;
	_pub_est_status.get().pos_vert_accuracy = _pub_gpos.get().epv;

	_pub_est_status.update();
}

void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = _xLowPass.getState();
	map_projection_reproject(&_map_ref, xLP(X_x), xLP(X_y), &lat, &lon);
	float alt = -xLP(X_z) + _altOrigin;

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float vxy_stddev = sqrtf(_P(X_vx, X_vx) + _P(X_vy, X_vy));
	float epv = sqrtf(_P(X_z, X_z));
	float eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));
	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (vxy_stddev < _vxy_pub_thresh.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    PX4_ISFINITE(xLP(X_vx)) && PX4_ISFINITE(xLP(X_vy)) &&
	    PX4_ISFINITE(xLP(X_vz))) {
		_pub_gpos.get().timestamp = _timeStamp;
		_pub_gpos.get().time_utc_usec = _sub_gps.get().time_utc_usec;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().vel_n = xLP(X_vx);
		_pub_gpos.get().vel_e = xLP(X_vy);
		_pub_gpos.get().vel_d = xLP(X_vz);
		_pub_gpos.get().yaw = _eul(2);
		_pub_gpos.get().eph = eph;
		_pub_gpos.get().epv = epv;
		_pub_gpos.get().terrain_alt = _altOrigin - xLP(X_tz);
		_pub_gpos.get().terrain_alt_valid = _validTZ;
		_pub_gpos.get().dead_reckoning = !_validXY && !_xyTimeout;
		_pub_gpos.get().pressure_alt = _sub_sensor.get().baro_alt_meter;
		_pub_gpos.update();
	}
}

void BlockLocalPositionEstimator::initP()
{
	_P.setZero();
	// initialize to twice valid condition
	_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	_P(X_vx, X_vx) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	_P(X_vy, X_vy) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	// use vxy thresh for vz init as well
	// 同时使用 vxy 初始化vz
	_P(X_vz, X_vz) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	// initialize bias uncertainty to small values to keep them stable
	// 初始化加速度偏差的不确定度为一个很小的值以使其保持稳定
	_P(X_bx, X_bx) = 1e-6;
	_P(X_by, X_by) = 1e-6;
	_P(X_bz, X_bz) = 1e-6;
	_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

void BlockLocalPositionEstimator::initSS()
{
	initP();

	// dynamics matrix
	_A.setZero();
	// derivative of position is velocity
	// 位置的微分得到速度
	_A(X_x, X_vx) = 1;
	_A(X_y, X_vy) = 1;
	_A(X_z, X_vz) = 1;

	// input matrix
	_B.setZero();
	_B(X_vx, U_ax) = 1;
	_B(X_vy, U_ay) = 1;
	_B(X_vz, U_az) = 1;

	// update components that depend on current state
	// 根据当前状态更新组件
	updateSSStates(); // 更新状态矩阵 A
	updateSSParams(); // 更新噪声矩阵 R Q
}

void BlockLocalPositionEstimator::updateSSStates()
{
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	// 速度的微分为加速度计测得的加速度(用于输入矩阵) -> 机体系下的偏移
	/*    DCM_e = [i_e j_e k_e] 
	 *          = [ I_b; 
	 *              J_b; 
	 *              K_b ]
	 */
	// i_e、j_e、k_e是机体系三轴向量在地理系中的表示
	// I_b、J_b、K_b是地理系三轴向量在机体系中的表示
	//
	//  加速度计模型 a = R^T * ( v_dot - g) + a_b
	// 低频状态下 v_dot远小于g
	// va = - R^T * e
	_A(X_vx, X_bx) = -_R_att(0, 0); // I_b
	_A(X_vx, X_by) = -_R_att(0, 1);
	_A(X_vx, X_bz) = -_R_att(0, 2);

	_A(X_vy, X_bx) = -_R_att(1, 0);
	_A(X_vy, X_by) = -_R_att(1, 1);
	_A(X_vy, X_bz) = -_R_att(1, 2);

	_A(X_vz, X_bx) = -_R_att(2, 0);
	_A(X_vz, X_by) = -_R_att(2, 1);
	_A(X_vz, X_bz) = -_R_att(2, 2);
}

void BlockLocalPositionEstimator::updateSSParams()
{
	// input noise covariance matrix
	// 输入噪声协方差矩阵
	_R.setZero();
	_R(U_ax, U_ax) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	_R(U_ay, U_ay) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	_R(U_az, U_az) = _accel_z_stddev.get() * _accel_z_stddev.get();

	// process noise power matrix
	// 过程噪声能量矩阵
	_Q.setZero();
	float pn_p_sq = _pn_p_noise_density.get() * _pn_p_noise_density.get();
	float pn_v_sq = _pn_v_noise_density.get() * _pn_v_noise_density.get();
	_Q(X_x, X_x) = pn_p_sq;
	_Q(X_y, X_y) = pn_p_sq;
	_Q(X_z, X_z) = pn_p_sq;
	_Q(X_vx, X_vx) = pn_v_sq;
	_Q(X_vy, X_vy) = pn_v_sq;
	_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	// 噪声是在机体系下的，但是由于成分相同，暂时忽略这一点
	float pn_b_sq = _pn_b_noise_density.get() * _pn_b_noise_density.get();
	_Q(X_bx, X_bx) = pn_b_sq;
	_Q(X_by, X_by) = pn_b_sq;
	_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	// 地势随机游走噪声，随速度缩放
	float pn_t_noise_density =
		_pn_t_noise_density.get() +
		(_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;

}

void BlockLocalPositionEstimator::predict()
{
	// if can't update anything, don't propagate
	// state or covariance
	if (!_validXY && !_validZ) { return; }

	if (integrate) {
		matrix::Quaternion<float> q(&_sub_att.get().q[0]);
		_eul = matrix::Euler<float>(q);
		_R_att = matrix::Dcm<float>(q);
		Vector3f a(_sub_sensor.get().accelerometer_m_s2);
		// note, bias is removed in dynamics function
		// 在动力学函数中去除了偏差
		_u = _R_att * a; // 非线性系统的输入向量   加速度
		_u(U_az) += 9.81f; // add g

	} else {
		_u = Vector3f(0, 0, 0);
	}

	// update state space based on new states
	// 基于新的状态更新状态空间矩阵 A
	updateSSStates(); // 用 _R_att 更新 A

	// 连续时间的卡尔曼滤波器预测值
	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	// 四元数微分求解，四阶龙格库塔法
	float h = getDt();
	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u); // A * _x + B * _u
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u); // A * (x + k1 * h/2) + B * u
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);  // A * (x + k2 * h/2) + B * u
	k4 = dynamics(h, _x + k3 * h, _u); // A * ( x + k3 * h) + B * u
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6); // 状态增量

	// propagate
	// 传播
	correctionLogic(dx); // 逻辑校正，积分量逻辑判断
	_x += dx; // 状态估计
	Matrix<float, n_x, n_x> dP = (_A * _P + _P * _A.transpose() +
				      _B * _R * _B.transpose() + _Q) * getDt(); // 过程噪声协方差矩阵增量
	covPropagationLogic(dP); // 逻辑校正
	_P += dP; // 状态协方差估计

	_xLowPass.update(_x);
	_aglLowPass.update(agl());
}

int BlockLocalPositionEstimator::getDelayPeriods(float delay, uint8_t *periods)
{
	float t_delay = 0;
	uint8_t i_hist = 0;

	for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > delay) {
			break;
		}
	}

	*periods = i_hist;

	if (t_delay > DELAY_MAX) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] delayed data too old: %8.4f", double(t_delay));
		return -1;
	}

	return OK;
}
