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
#include "fauav_ctrl.hpp"
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

FauavCtrl::FauavCtrl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}
FauavCtrl::~FauavCtrl()
{
	perf_free(_loop_perf);
}

/**
 * @description: 通过_local_pos_sub回调启动Run
 */
bool
FauavCtrl::init()
{
	if (!_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("Fauav_ctrl started!");
	ScheduleNow();

	return true;
}

void
FauavCtrl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	bool arm = false;

	/* check for updates in other topics */
	_ctrl_mode_sub.update(&_vehicle_control_mode);
	_land_detected_sub.update(&_vehicle_land_detected);
	arm = _vehicle_control_mode.flag_armed;


	vehicle_attitude_s att{};
	vehicle_attitude_setpoint_s att_sp{};
	pos_onmi_s pos_onmi{};
	vehicle_local_position_s pos{};
	vehicle_local_position_setpoint_s pos_sp{};
	manual_control_setpoint_s	manual_sp {};	/**< manual control setpoint */

	_manual_control_setpoint_sub.update(&manual_sp);
	_pos_sp_sub.update(&pos_sp);
	_pos_onmi_sub.update(&pos_onmi);_att_sp_sub.update(&att_sp);
	// PX4_INFO("pos onim %f %f %f\n\n",(double)pos_onmi.pos[0],(double)pos_onmi.pos[1],(double)pos_onmi.pos[2]);
	// run controller on position & attitude updates
	if (_pos_sub.update(&pos) &&_att_sub.update(&att)) {
		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((pos.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = pos.timestamp;

		_control.setState(pos, att); // Set position and attitude states
		_control.setInputSetpoint(pos_sp,manual_sp,att_sp);
		// if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_position_enabled) {
		// 	// PX4_INFO("manual control enable!\n");
		// 	manual_control_setpoint_s	_manual_control_setpoint {};	/**< manual control setpoint */
		// 	vehicle_local_position_setpoint_s manual_pos_sp {};

		// 	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

		// 	manual_pos_sp.x = pos_sp.x + _manual_control_setpoint.x * 5;
		// 	manual_pos_sp.y = pos_sp.y + _manual_control_setpoint.y * 5;
		// 	_control.setInputSetpoint(manual_pos_sp, att_sp);
		// } else {
		// 	// PX4_INFO("position control enable!\n");
		// }

		const Vector3f thrust_onmi = _control.thrust_update(arm, dt);

		const Quatf q{att.q};
		const Vector3f torque_onmi = _control.torque_update(arm, q, att_sp.roll_body, att_sp.pitch_body,
					     att_sp.yaw_body, dt);
		/* 发布力和力矩Topic消息 */
		publishThrustSetpoint_onmi(thrust_onmi, pos_onmi.timestamp);
		publishTorqueSetpoint_onmi(torque_onmi, pos_onmi.timestamp);

	}

	perf_end(_loop_perf);
}

void
FauavCtrl::publishTorqueSetpoint_onmi(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	torque_sp_s		torque_onmi_sp {};
	torque_onmi_sp.timestamp = hrt_absolute_time();
	torque_onmi_sp.timestamp_sample = timestamp_sample;

	torque_onmi_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	torque_onmi_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	torque_onmi_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_torque_sp_pub.publish(torque_onmi_sp);
}

void
FauavCtrl::publishThrustSetpoint_onmi(const matrix::Vector3f &thrust_sp, const hrt_abstime &timestamp_sample)
{
	thrust_sp_s 		thrust_setpoint {};
	thrust_setpoint.timestamp = hrt_absolute_time();
	thrust_setpoint.timestamp_sample = timestamp_sample;

	thrust_setpoint.xyz[0] = (PX4_ISFINITE(thrust_sp(0))) ? thrust_sp(0) : 0.0f;
	thrust_setpoint.xyz[1] = (PX4_ISFINITE(thrust_sp(1))) ? thrust_sp(1) : 0.0f;
	thrust_setpoint.xyz[2] = (PX4_ISFINITE(thrust_sp(2))) ? (thrust_sp(2) > 0) ? thrust_sp(2) : 5.0f : 5.0f;

	_thrust_sp_pub.publish(thrust_setpoint);
}

int
FauavCtrl::task_spawn(int argc, char *argv[])
{
	FauavCtrl *instance = new FauavCtrl();

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
FauavCtrl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FauavCtrl::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

int FauavCtrl::print_usage(const char *reason)
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

extern "C" __EXPORT int fauav_ctrl_main(int argc, char *argv[]);
int fauav_ctrl_main(int argc, char *argv[])
{
	return FauavCtrl::main(argc, argv);
}
