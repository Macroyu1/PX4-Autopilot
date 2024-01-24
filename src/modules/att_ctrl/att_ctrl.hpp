/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-10-18 14:35:25
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-24 15:05:56
 * @FilePath: /PX4-Autopilot/src/modules/att_ctrl/att_ctrl.hpp
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

#include <AttCtrl.hpp>
/////////////////////////////////////////////////////
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
#include <uORB/topics/position_setpoint_onmi.h>
#include <uORB/topics/fault.h>

#include<lib/ladrc/ladrc.hpp>



using namespace time_literals;

class Att_Ctrl : public ModuleBase<Att_Ctrl>, public ModuleParams, public px4::ScheduledWorkItem
{
	public:
		Att_Ctrl();

		virtual ~Att_Ctrl();

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

		/**
		 * initialize some vectors/matrices from parameters
		 */
		void		parameters_updated();

		AttCtrl _control; /** 创建AttCtrl类的实例*/

		/**
		 * @description: 发布扭矩设定点话题
		 * @param {Vector3f} &torque_sp
		 * @param {hrt_abstime} &timestamp_sample
		 * @return {*}
		 */
		void publishTorqueSetpoint_onmi(const matrix::Vector3f &torque_sp,const hrt_abstime &timestamp_sample);

		/**
		 * @description: 发布推力设定点话题
		 * @param {Vector3f} &thrust_sp
		 * @param {hrt_abstime} &timestamp_sample
		 * @return {*}
		 */
		void publishThrustSetpoint_onmi(const matrix::Vector3f &thrust_sp,const hrt_abstime &timestamp_sample);

		//Inputs
		uORB::SubscriptionCallbackWorkItem 	_att_sub{this, ORB_ID(vehicle_attitude)};
		uORB::Subscription 			_pos_sub {ORB_ID(vehicle_local_position)};	/**< vehicle local position */
		uORB::Subscription 			_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};  /**< vehicle attitude setpoint subscription */
		uORB::Subscription 			_pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
		uORB::Subscription 			_ctrl_mode_sub {ORB_ID(vehicle_control_mode)};
		uORB::Subscription 			_land_detected_sub {ORB_ID(vehicle_land_detected)};
		uORB::Subscription 			_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
		uORB::Subscription _fault_sub{ORB_ID(fault)};
		//Outputs
		uORB::Publication<torque_sp_s> 		 _torque_sp_pub{ORB_ID(torque_sp)};//torque_onmi
		uORB::Publication<thrust_sp_s> 		 _thrust_sp_pub{ORB_ID(thrust_sp)};//thrust_onmi
		uORB::Publication<position_setpoint_onmi_s> 		 _pos_sp_onmi_pub{ORB_ID(position_setpoint_onmi)};//thrust_onmi
		vehicle_control_mode_s _vehicle_control_mode {};
		vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
		};

		matrix::Vector3f pos_gain;

/* 		DEFINE_PARAMETERS(
		(ParamFloat<px4::params::X_P>)         _param_x_p,
		(ParamFloat<px4::params::X_WC>)        _param_x_wc,
		(ParamInt<px4::params::X_C2O>)         _param_x_c2o,
		(ParamFloat<px4::params::X_B0>)        _param_x_b0,

		(ParamFloat<px4::params::Y_P>)         _param_y_p,
		(ParamFloat<px4::params::Y_WC>)        _param_y_wc,
		(ParamInt<px4::params::Y_C2O>)         _param_y_c2o,
		(ParamFloat<px4::params::Y_B0>)        _param_y_b0,

		(ParamFloat<px4::params::Z_P>)         _param_z_p,
		(ParamFloat<px4::params::Z_WC>)        _param_z_wc,
		(ParamInt<px4::params::Z_C2O>)         _param_z_c2o,
		(ParamFloat<px4::params::Z_B0>)        _param_z_b0
		) */









		// Performance (perf) counters
		perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")}; /**< loop duration performance counter */

		hrt_abstime _last_run{0};
};
