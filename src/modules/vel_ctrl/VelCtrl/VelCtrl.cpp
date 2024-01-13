/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2024-01-09 17:19:12
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-09 20:51:46
 * @FilePath: /PX4-Autopilot/src/modules/vel_ctrl/VelCtrl/VelCtrl.cpp
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
 * @file VelCtrl.cpp
 * @copyright HongYu Fu
 * ADRC control lib for fully-actuated uav.
 */

#include <VelCtrl.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
using namespace matrix;

void
VelCtrl::setState(const vehicle_attitude_s &local_att)
{
	ControlStates states;
	const Quatf q{local_att.q};
	// only set position states if valid and finite
	if (PX4_ISFINITE(Eulerf(q).phi()) && PX4_ISFINITE(Eulerf(q).theta()) && Eulerf(q).psi()) {
		states.attitued(0) = Eulerf(q).phi();
		states.attitued(1) = Eulerf(q).theta();
		states.attitued(2) = Eulerf(q).psi();
	} else {
		states.attitued(0) = NAN;
		states.attitued(1) = NAN;
		states.attitued(2) = NAN;
	}

	_att = states.attitued;
	// PX4_INFO("local_position  %f %f %f\n",(double)local_pos.x,(double)local_pos.y,(double)local_pos.z);
}

void
VelCtrl::setInputSetpoint(const vehicle_attitude_setpoint_s &att_setpoint)
{
	_att_sp = Vector3f(att_setpoint.roll_body,att_setpoint.pitch_body,att_setpoint.yaw_body);
}

matrix::Vector3f
VelCtrl::torque_update(bool takeoff,const matrix::Quatf &q,float roll,float pitch,float yaw,const float dt)
{
	static LADRC Phi(1.3,2);static LADRC Theta(1.3,2);static LADRC Psai(2,2);
	Vector3f torque,angle,angle_sp;
	// NED 2 ENU
	angle(0) = Eulerf(q).phi();
	angle(1) = Eulerf(q).theta();
	angle(2) = -Eulerf(q).psi() + (float)M_PI/2.f + M_PI/6.f;

	/* angle_sp(0) = pitch;
	angle_sp(1) = -roll;
	angle_sp(2) = -yaw + (float)M_PI/2.f; */
	angle_sp(0) = 0.175;
	angle_sp(1) = 0.175;
	angle_sp(2) = 0;

	if(takeoff){
		torque(0) = Phi.ADRC_Run(angle(0),angle_sp(0),dt,-10,10);
		torque(1) = Theta.ADRC_Run(angle(1),angle_sp(1),dt,-10,10);
		torque(2) = Psai.ADRC_Run(angle(2),angle_sp(2),dt,-5,5);
		Vector3f error = R2D(angle_sp - angle);
		// PX4_INFO("Attit/ude error: %f %f\n\n",(double)angle(0),(double)angle(1));
		// PX4_INFO("Attitude error: %f %f %f\n\n",(double)error(0),(double)error(1),(double)error(2));
		// Theta.ADRC_Log(1);
	}else{
		torque(0) = Phi.ADRC_Reset();
		torque(1) = Theta.ADRC_Reset();
		torque(2) = Psai.ADRC_Reset();
	}
	bool log = 0;
	if(log){
		// PX4_INFO("%d\n",takeoff);
		PX4_INFO("phi %f %f %f\n\n",(double)angle(0),(double)angle_sp(0),(double)torque(0));
		PX4_INFO("theta %f %f %f\n\n",(double)angle(1),(double)angle_sp(1),(double)torque(1));
	}
	return torque;
}

matrix::Vector3f
VelCtrl::R2D(matrix::Vector3f error)
{
	Vector3f error_d;
	error_d(0) = error(0)*90.f/3.14f;
	error_d(1) = error(1)*90.f/3.14f;
	error_d(2) = error(2)*90.f/3.14f;
	return error_d;
}
