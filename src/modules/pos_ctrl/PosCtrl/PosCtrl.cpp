/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2024-01-09 15:32:07
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-24 10:22:55
 * @FilePath: /PX4-Autopilot/src/modules/pos_ctrl/PosCtrl/PosCtrl.cpp
 * @Description:
 *
 * Copyright (c) 2024 by HongYu Fu, All Rights Reserved.
 */
/*
 *                                                     __----~~~~~~~~~~~------___
 *                                    .  .   ~~//====......          __--~ ~~
 *                    -.            \_|//     |||\\  ~~~~~~::::... /~
 *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
 *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
 *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *           '         ~-|      /|    |-~\~~       __--~~
 *                       |-~~-_/ |    |   ~\_   _-~            /\
 *                            /  \     \__   \/~                \__
 *                        _--~ _/ | .-~~____--~-/                  ~~==.
 *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                  -_     ~\      ~~---l__i__i__i--~~_/
 *                                  _-~-__   ~)  \--______________--~~
 *                                //.-~~~-~_--~- |-------~~~~~~~~
 *                                       //.-~~~--\
 *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *                               神兽保佑            永无BUG
 */

/**
 * @file PosCtrl.cpp
 * @copyright HongYu Fu
 * ADRC control lib for fully-actuated uav.
 */

#include <PosCtrl.hpp>
#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <geo/geo.h>
using namespace matrix;

PosCtrl::PosCtrl()
{
	/* LADRC X(_vel_wc(0),_vel_b0(0),_vel_c20(0));
	LADRC Y(_vel_wc(1),_vel_b0(1),_vel_c20(1));
	LADRC Z(_vel_wc(2),_vel_b0(2),_vel_c20(2)); */
}
void PosCtrl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PosCtrl::setADRCparms(const Vector3f &wc, const Vector3f &b0, const Vector3f &c20)
{
	_vel_wc  = wc;
	_vel_b0  = b0;
	_vel_c20 = c20;
}

void PosCtrl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = 2;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
	// PX4_INFO("lim : %f\n\n",(double)_lim_vel_horizontal);
}

void PosCtrl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PosCtrl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void
PosCtrl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint,const manual_control_setpoint_s &manual_setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

matrix::Vector3f
PosCtrl::setHoverThrust(const float dt,const bool flying)
{
	if(!flying){
		_positionControl();
		Vector3f vel_error = _vel_sp - _vel;
		_thr_sp(2) = -(vel_error(2)*5 + _vel_int(2) + _vel_dot(2)*1);
		// PX4_INFO("%f %f %f\n",(double)_vel_sp(0),(double)_vel_sp(1),(double)_vel_sp(2));
		// Make sure integral doesn't get NAN
		ControlMath::setZeroIfNanVector3f(vel_error);
		// Update integral part of velocity control
		_vel_int(2) += vel_error(2)*3 * dt;
	}
	PX4_INFO("setHoverThrust : %f\n\n",(double)_thr_sp(2));
	return _thr_sp;
}

matrix::Vector3f
PosCtrl::thrust_update(const bool want_takeoff,const float dt)
{
	_positionControl();
	// PX4_INFO("%f %f %f\n",(double)_vel_sp(0),(double)_vel_sp(1),(double)_vel_sp(2));
	_velocityControl(want_takeoff,dt);
	// PX4_INFO("thr_sp : %f %f %f\n",(double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));

	return _thr_sp;
}
void PosCtrl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PosCtrl::_velocityControl(const bool want_takeoff,const float dt)
{
	static LADRC X(1.4,1,3);static LADRC Y(1.2,1,3);static LADRC Z(3.3,1,6);
	// ADRC velocity control
	if(want_takeoff){
		// _thr_sp = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
		// _thr_sp(1) = (vel_error(1)*2 + _vel_int(1) + _vel_dot(1)*0.5);
		// _thr_sp(0) = X.ADRC_Run(_vel(0),_vel_sp(0),dt,-10,10);
		// _thr_sp(0) = Y.ADRC_Run(_vel(1),_vel_sp(1),dt,-10,10);
		// _thr_sp(1) = X.ADRC_Run(_vel(0),_vel_sp(0),dt,-10,10);
		_thr_sp(2) = Z.ADRC_Run(-_vel(2),-_vel_sp(2),dt,5,15);

		PX4_INFO("X : %f %f %f\n\n",(double)_vel(0),(double)_vel_sp(0),(double)_thr_sp(0));
		PX4_INFO("Y : %f %f %f\n\n",(double)_vel(1),(double)_vel_sp(1),(double)_thr_sp(1));
		PX4_INFO("Z : %f %f %f\n\n",(double)_vel(2),(double)-_vel_sp(2),(double)_thr_sp(2));

		PX4_INFO("error : %f %f %f\n\n",(double)_pos(0)-_pos_sp(0),(double)_pos(1)-_pos_sp(1),(double)_pos(2)-_pos_sp(2));
	}else{
		_thr_sp.setZero();
		X.ADRC_Reset();
		Y.ADRC_Reset();
		Z.ADRC_Reset();
	}
}

matrix::Vector3f
PosCtrl::R2D(matrix::Vector3f error)
{
	Vector3f error_d;
	error_d(0) = error(0)*90.f/3.14f;
	error_d(1) = error(1)*90.f/3.14f;
	error_d(2) = error(2)*90.f/3.14f;
	return error_d;
}
