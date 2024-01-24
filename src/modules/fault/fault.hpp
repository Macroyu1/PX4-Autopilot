/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-10-18 14:35:25
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-24 14:54:49
 * @FilePath: /PX4-Autopilot/src/modules/fault/fault.hpp
 * @Description:Control for fully-actuated uav
 *
 * Copyright (c) 2023 by HongYu Fu, All Rights Reserved.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/uORB.h>
#include <perf/perf_counter.h>

/////////////////////////////////////////////////////
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/torque_sp.h>
#include <uORB/topics/thrust_sp.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/fault.h>



using namespace time_literals;

class FAULT : public ModuleBase<FAULT>, public ModuleParams, public px4::ScheduledWorkItem
{
	public:
		FAULT();

		virtual ~FAULT();

		/** @see ModuleBase */
		static int task_spawn(int argc, char *argv[]);

		/** @see ModuleBase */
		static int custom_command(int argc, char *argv[]);

		/** @see ModuleBase */
		static int print_usage(const char *reason = nullptr);

		bool init();

		int print_status() override;
	private:
		void Run() override;


		//Inputs
		uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< vehicle torque setpoint subscription */
		uORB::SubscriptionCallbackWorkItem _vehicle_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint)};	 /**< vehicle thrust setpoint subscription */
		uORB::Subscription 			_att_sub{ORB_ID(vehicle_attitude)};
		uORB::Subscription 			_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};  /**< vehicle attitude setpoint subscription */
		uORB::Subscription 			_pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
		uORB::Subscription 			_ctrl_mode_sub {ORB_ID(vehicle_control_mode)};
		uORB::Subscription 			_land_detected_sub {ORB_ID(vehicle_land_detected)};
		uORB::Subscription 			_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
		//Outputs
		uORB::Publication<fault_s> 		 _fault_pub{ORB_ID(fault)};//torque_onmi

		// Performance (perf) counters
		perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")}; /**< loop duration performance counter */

		hrt_abstime _last_run{0};
};
