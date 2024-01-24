/*
 *
 * 　　┏┓　　　┏┓+ +
 * 　┏┛┻━━━┛┻┓ + +
 * 　┃　　　　　　　┃ 　
 * 　┃　　　━　　　┃ ++ + + +
 *  ████━████ ┃+
 * 　┃　　　　　　　┃ +
 * 　┃　　　┻　　　┃
 * 　┃　　　　　　　┃ + +
 * 　┗━┓　　　┏━┛
 * 　　　┃　　　┃　　　　　　　　　　　
 * 　　　┃　　　┃ + + + +
 * 　　　┃　　　┃
 * 　　　┃　　　┃ +  神兽保佑
 * 　　　┃　　　┃    代码无bug　　
 * 　　　┃　　　┃　　+　　　　　　　　　
 * 　　　┃　 　　┗━━━┓ + +
 * 　　　┃ 　　　　　　　┣┓
 * 　　　┃ 　　　　　　　┏┛
 * 　　　┗┓┓┏━┳┓┏┛ + + + +
 * 　　　　┃┫┫　┃┫┫
 * 　　　　┗┻┛　┗┻┛+ + + +
 *
 */

/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-10-18 14:35:25
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2023-10-23 15:01:52
 * @FilePath: /PX4-Autopilot/src/modules/fauav_ctrl/fauav_ctrl.cpp
 * @Description: fauav_ctrl.cpp
 *
 * Copyright (c) 2023 by HongYu Fu, All Rights Reserved.
 */
#include "pos_ctrl.hpp"
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

Pos_Ctrl::Pos_Ctrl() :
	ModuleParams(nullptr),
	SuperBlock(nullptr, "MPC"),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{
}
Pos_Ctrl::~Pos_Ctrl()
{
	perf_free(_loop_perf);
}

void
Pos_Ctrl::parameters_update()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	_control.setPositionGains(Vector3f(_param_x_p.get(), _param_y_p.get(), _param_z_p.get()));
	_control.setVelocityGains(
			Vector3f(_param_xy_vel_p.get(), _param_xy_vel_i.get(), _param_xy_vel_d.get()),
			Vector3f(_param_xy_vel_p.get(), _param_xy_vel_i.get(), _param_xy_vel_d.get()),
			Vector3f(_param_z_vel_p.get(), _param_z_vel_i.get(), _param_z_vel_d.get()));

	_control.setADRCparms(
			Vector3f(_param_x_wc.get(), _param_x_b0.get(), _param_x_c2o.get()),
			Vector3f(_param_y_wc.get(), _param_y_b0.get(), _param_y_c2o.get()),
			Vector3f(_param_z_wc.get(), _param_z_b0.get(), _param_z_c2o.get()));
}

void
Pos_Ctrl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update();

	perf_begin(_loop_perf);

	/* check for updates in other topics */
	_ctrl_mode_sub.update(&_vehicle_control_mode);
	_land_detected_sub.update(&_vehicle_land_detected);
	bool arm = _vehicle_control_mode.flag_armed;

	vehicle_local_position_s pos{};
	vehicle_local_position_setpoint_s pos_sp{};
	manual_control_setpoint_s	manual_sp {};	/**< manual control setpoint */

	// run controller on position & attitude updates
	if (_pos_sub.update(&pos)) {
		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((pos.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = pos.timestamp;
		// set _dt in controllib Block for BlockDerivative
		setDt(dt);

		_manual_control_setpoint_sub.update(&manual_sp);
		_pos_sp_sub.update(&pos_sp);

/* 		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;
			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		} */
		//设定当前位置
		PositionControlStates states{set_vehicle_states(pos)};

		//轨迹跟踪
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {

			const bool is_trajectory_setpoint_updated = _trajectory_setpoint_sub.update(&pos_sp);

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);
			const bool want_takeoff =  _vehicle_control_mode.flag_armed && _vehicle_land_detected.landed
						    && hrt_elapsed_time(&pos_sp.timestamp) < 1_s;

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, pos.timestamp);

			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);

			if (is_trajectory_setpoint_updated) {
				// make sure takeoff ramp is not amended by acceleration feed-forward
				if (!flying) {
					pos_sp.acceleration[2] = NAN;
					// hover_thrust maybe reset on takeoff
				}

				const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
				const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

				if (not_taken_off || flying_but_ground_contact) {
					// we are not flying yet and need to avoid any corrections
					reset_setpoint_to_nan(pos_sp);
					Vector3f(0.f, 0.f, 5.f).copyTo(pos_sp.acceleration); // High downwards acceleration to make sure there's no thrust
				}
			}

			_control.setState(states);

			_control.setInputSetpoint(pos_sp,manual_sp);
			_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());

			fault_s fault{};
			_fault_sub.update(&fault);

			if (fault.fault_flag == 1) {
				thrust_onmi = _control.thrust_update_fault(arm,dt);

			} else {
				thrust_onmi = _control.thrust_update(arm,dt);
			}



			/* 发布力和力矩Topic消息 */
			publishThrustSetpoint_onmi(thrust_onmi, pos.timestamp);
		}

	}

	perf_end(_loop_perf);
}

PositionControlStates Pos_Ctrl::set_vehicle_states(const vehicle_local_position_s &local_pos)
{
	PositionControlStates states;

	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		states.position(0) = local_pos.x;
		states.position(1) = local_pos.y;

	} else {
		states.position(0) = NAN;
		states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		states.position(2) = local_pos.z;

	} else {
		states.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		states.velocity(0) = local_pos.vx;
		states.velocity(1) = local_pos.vy;
		states.acceleration(0) = _vel_x_deriv.update(local_pos.vx);
		states.acceleration(1) = _vel_y_deriv.update(local_pos.vy);

	} else {
		states.velocity(0) = NAN;
		states.velocity(1) = NAN;
		states.acceleration(0) = NAN;
		states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		states.velocity(2) = local_pos.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	states.yaw = local_pos.heading;

	return states;
}

void Pos_Ctrl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

void
Pos_Ctrl::publishThrustSetpoint_onmi(const matrix::Vector3f &thrust_sp, const hrt_abstime &timestamp_sample)
{
	thrust_sp_s 		thrust_setpoint {};
	thrust_setpoint.timestamp = hrt_absolute_time();
	thrust_setpoint.timestamp_sample = timestamp_sample;

	thrust_setpoint.xyz[0] = (PX4_ISFINITE(thrust_sp(0))) ? thrust_sp(0) : 0.0f;
	thrust_setpoint.xyz[1] = (PX4_ISFINITE(thrust_sp(1))) ? thrust_sp(1) : 0.0f;
	thrust_setpoint.xyz[2] = (PX4_ISFINITE(thrust_sp(2))) ? (thrust_sp(2) > 0) ? thrust_sp(2) : 5.0f : 5.0f;

	_thrust_sp_pub.publish(thrust_setpoint);
}

/**
 * @description: 通过_local_pos_sub回调启动Run
 */
bool
Pos_Ctrl::init()
{
	if (!_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("POS_Ctrl started!");
	ScheduleNow();

	return true;
}

int
Pos_Ctrl::task_spawn(int argc, char *argv[])
{
	Pos_Ctrl *instance = new Pos_Ctrl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
Pos_Ctrl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Pos_Ctrl::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

int Pos_Ctrl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the fully-actuated uav control!

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fauav_ctrl", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");

	return 0;
}

extern "C" __EXPORT int pos_ctrl_main(int argc, char *argv[]);
int pos_ctrl_main(int argc, char *argv[])
{
	return Pos_Ctrl::main(argc, argv);
}
