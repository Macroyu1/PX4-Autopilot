/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2024-01-09 17:19:12
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-09 20:54:45
 * @FilePath: /PX4-Autopilot/src/modules/vel_ctrl/vel_ctrl.cpp
 * @Description:
 *
 * Copyright (c) 2024 by HongYu Fu, All Rights Reserved.
 */
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

#include "vel_ctrl.hpp"
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

Vel_Ctrl::Vel_Ctrl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}
Vel_Ctrl::~Vel_Ctrl()
{
	perf_free(_loop_perf);
}

void
Vel_Ctrl::parameters_updated()
{


}

void
Vel_Ctrl::Run()
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

	// run controller on position & Velitude updates
	if (_att_sp_sub.update(&att_sp) &&_att_sub.update(&att)) {
		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((att.timestamp - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = att.timestamp;

		_control.setState(att); // Set position and Velitude states
		_control.setInputSetpoint(att_sp);

		const Quatf q{att.q};
		const Vector3f torque_onmi = _control.torque_update(arm, q, att_sp.roll_body, att_sp.pitch_body,
					     att_sp.yaw_body, dt);
		/* 发布力和力矩Topic消息 */
		// publishThrustSetpoint_onmi(thrust_onmi, pos.timestamp);

	}

	perf_end(_loop_perf);
}

void
Vel_Ctrl::publishTorqueSetpoint_onmi(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
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
Vel_Ctrl::publishThrustSetpoint_onmi(const matrix::Vector3f &thrust_sp, const hrt_abstime &timestamp_sample)
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
Vel_Ctrl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("Vel_Ctrl started!");
	ScheduleNow();

	return true;
}

int
Vel_Ctrl::task_spawn(int argc, char *argv[])
{
	Vel_Ctrl *instance = new Vel_Ctrl();

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
Vel_Ctrl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Vel_Ctrl::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

int Vel_Ctrl::print_usage(const char *reason)
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

extern "C" __EXPORT int vel_ctrl_main(int argc, char *argv[]);
int vel_ctrl_main(int argc, char *argv[])
{
	return Vel_Ctrl::main(argc, argv);
}
