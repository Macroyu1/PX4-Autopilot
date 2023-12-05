/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-10-23 08:47:53
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2023-11-02 14:59:23
 * @FilePath: /PX4-Autopilot/src/modules/fauav_ctrl/FaCtrl/FaCtrl.hpp
 * @Description:	ADRC control lib for fully-actuated uav.
 *
 * Copyright (c) 2023 by HongYu Fu, All Rights Reserved.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>
#include<lib/ladrc/ladrc.hpp>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/position_setpoint_onmi.h>
#include <uORB/topics/manual_control_setpoint.h>

struct ControlStates {
	matrix::Vector3f position;
	matrix::Vector3f attitued;
	float yaw;
};

class FaCtrl
{
public:
	FaCtrl() = default;
	~FaCtrl() = default;

	/**
	 * @description: 获取当前状态
	 * @param {vehicle_local_position_s} &local_pos		当前位置结构体
	 * @param {vehicle_attitude_s} &local_att		当前姿态结构体
	 * @return {*}
	 */
	void setState(const vehicle_local_position_s &local_pos,const vehicle_attitude_s &local_att);

	/**
	 * @description: 获取期望状态
	 * @param {vehicle_local_position_setpoint_s} &pos_setpoint	位置设定点结构体
	 * @param {vehicle_attitude_setpoint_s} &att_setpoint		姿态设定点结构体
	 * @return {*}
	 */
	void setInputSetpoint(const vehicle_local_position_setpoint_s &pos_setpoint,const manual_control_setpoint_s &manual_setpoint,const vehicle_attitude_setpoint_s &att_setpoint);

	/**
	 * @description: 更新扭矩向量
	 * @param {bool} takeoff	飞行状态
	 * @param {Quatf} &q		当前状态的四元数
	 * @param {float} roll		roll设定值
	 * @param {float} pitch		pitch设定值
	 * @param {float} yaw		yaw设定值
	 * @param {float} dt		离散时间
	 * @return {*}			扭矩向量
	 */
	matrix::Vector3f torque_update(bool takeoff,const matrix::Quatf &q,float roll,float pitch,float yaw,const float dt);

	/**
	 * @description: 		更新推力向量
	 * @param {bool} takeoff 	飞行状态
	 * @param {float} dt	 	离散时间
	 * @return {*}			推力向量
	 */
	matrix::Vector3f thrust_update(bool takeoff,const float dt);

	void getPositionSetpoint(position_setpoint_onmi_s &pos_setpoint) const;

private:

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _att; /**< current attitude */
	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _att_sp; /**< desired attitude */

};
