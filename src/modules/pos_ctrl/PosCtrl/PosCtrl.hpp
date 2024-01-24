/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-10-23 08:47:53
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-24 15:02:55
 * @FilePath: /PX4-Autopilot/src/modules/pos_ctrl/PosCtrl/PosCtrl.hpp
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

struct PositionControlStates  {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

class PosCtrl
{
public:
	PosCtrl();
	~PosCtrl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set the normalized hover thrust
	 * @param thrust [0.1, 0.9] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	matrix::Vector3f setHoverThrust(const float dt,const bool flying);

	/**
	 * @description: 获取当前状态
	 * @param {vehicle_local_position_s} &local_pos		当前位置结构体
	 * @param {vehicle_attitude_s} &local_att		当前姿态结构体
	 * @return {*}
	 */
	void setState(const PositionControlStates &states);

	/**
	 * @description: 获取期望状态
	 * @param {vehicle_local_position_setpoint_s} &pos_setpoint	位置设定点结构体
	 * @param {vehicle_attitude_setpoint_s} &att_setpoint		姿态设定点结构体
	 * @return {*}
	 */
	void setInputSetpoint(const vehicle_local_position_setpoint_s &pos_setpoint,const manual_control_setpoint_s &manual_setpoint);

	/**
	 * @description: 		更新推力向量
	 * @param {bool} takeoff 	飞行状态
	 * @param {float} dt	 	离散时间
	 * @return {*}			推力向量
	 */
	matrix::Vector3f thrust_update(const bool want_takeoff,const float dt);
	matrix::Vector3f thrust_update_fault(const bool want_takeoff,const float dt);

	matrix::Vector3f R2D(matrix::Vector3f error);

	void setADRCparms(const matrix::Vector3f &wc, const matrix::Vector3f &b0, const matrix::Vector3f &c20);

	void _positionControl();///< Position proportional control
	void _velocityControl(const bool want_takeoff,const float dt); ///< Velocity PID control
	void _velocityControl_fault(const bool want_takeoff,const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing
	void Reset();

private:
	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain
	matrix::Vector3f _vel_wc; ///<
	matrix::Vector3f _vel_b0; ///<
	matrix::Vector3f _vel_c20; ///<
	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */
	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */
	matrix::Vector3f r2d;
};
