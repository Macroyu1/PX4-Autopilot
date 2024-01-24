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
#include "fault.hpp"
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

FAULT::FAULT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}
FAULT::~FAULT()
{
	perf_free(_loop_perf);
}

/**
 * @description: 通过_local_pos_sub回调启动Run
 */
bool
FAULT::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	PX4_INFO("Fauav_ctrl started!");
	ScheduleNow();

	return true;
}

void
FAULT::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	fault_s fault{};
	fault.fault_flag = 1;
	_fault_pub.publish(fault);
	perf_end(_loop_perf);
}

int
FAULT::task_spawn(int argc, char *argv[])
{
	FAULT *instance = new FAULT();

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
FAULT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FAULT::print_status()
{
	perf_print_counter(_loop_perf);
	return 0;
}

int FAULT::print_usage(const char *reason)
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

extern "C" __EXPORT int fault_main(int argc, char *argv[]);
int fault_main(int argc, char *argv[])
{
	return FAULT::main(argc, argv);
}
