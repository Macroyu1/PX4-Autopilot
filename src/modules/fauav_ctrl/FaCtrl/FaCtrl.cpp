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
 * @file FaCtrl.cpp
 * @copyright HongYu Fu
 * ADRC control lib for fully-actuated uav.
 */

#include <FaCtrl.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
using namespace matrix;

void
FaCtrl::setState(const vehicle_local_position_s &local_pos,const vehicle_attitude_s &local_att)
{
	ControlStates states;
	const Quatf q{local_att.q};
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

	if (PX4_ISFINITE(Eulerf(q).phi()) && PX4_ISFINITE(Eulerf(q).theta()) && Eulerf(q).psi()) {
		states.attitued(0) = Eulerf(q).phi();
		states.attitued(1) = Eulerf(q).theta();
		states.attitued(2) = Eulerf(q).psi();
	} else {
		states.attitued(0) = NAN;
		states.attitued(1) = NAN;
		states.attitued(2) = NAN;
	}
	_vel(2) = local_pos.vz;
	_pos = states.position;
	_att = states.attitued;
	// PX4_INFO("local_position  %f %f %f\n",(double)local_pos.x,(double)local_pos.y,(double)local_pos.z);
}

void
FaCtrl::setInputSetpoint(const vehicle_local_position_setpoint_s &pos_setpoint,const manual_control_setpoint_s &manual_setpoint,const vehicle_attitude_setpoint_s &att_setpoint)
{
	_pos_sp = Vector3f(pos_setpoint.x, pos_setpoint.y, pos_setpoint.z);
	_att_sp = Vector3f(att_setpoint.roll_body,att_setpoint.pitch_body,att_setpoint.yaw_body);
}

matrix::Vector3f
FaCtrl::thrust_update(bool takeoff,const float dt)
{
	Vector3f thrust,pos_onmi,pos_onmi_sp;

	static LADRC X(1.4,2);static LADRC Y(1.7,2);static LADRC Z(2,0.5);

	pos_onmi_sp(0) = _pos_sp(0);
	pos_onmi_sp(1) = _pos_sp(1);
	pos_onmi(2) = _pos(2) > 0? 0 :-_pos(2);
	// pos_onmi_sp(2) = -_pos_sp(2) > 0 ? -_pos_sp(2) : 0;
	pos_onmi_sp(2) = 5;
	// PX4_INFO("states.position  %f %f %f\n",(double)_pos(0),(double)_pos(1),(double)pos_onmi(2));
	if(takeoff){
		thrust(0) = -X.ADRC_XY(_pos(0),pos_onmi_sp(0),_vel(0),dt,-10,10);
		// thrust(0) = 1;
		PX4_INFO("X : %f %f %f\n\n",(double)_pos(0),(double)pos_onmi_sp(0),(double)thrust(0));
		// thrust(1) = Y.ADRC_Run(_pos(1),pos_onmi_sp(1),dt,-10,10);
		// thrust(1) = 1;
		thrust(1) = Y.ADRC_XY(_pos(1),pos_onmi_sp(1),_vel(1),dt,-10,10);
		PX4_INFO("Y : %f %f %f\n\n",(double)_pos(1),(double)pos_onmi_sp(1),(double)thrust(1));
		// thrust(2) = Z.ADRC_Run(pos_onmi(2),pos_onmi_sp(2),dt,0.f,35.f);//给一个最小推力
		thrust(2) = Z.ADRC_Z(pos_onmi(2),pos_onmi_sp(2),_vel(2),dt,0.f,35.f);
		PX4_INFO("Z : %f %f %f\n\n",(double)pos_onmi(2),(double)pos_onmi_sp(2),(double)thrust(2));

		// Z.ADRC_Log(0);
	}else{
		thrust(0) = X.ADRC_Reset();
		thrust(1) = Y.ADRC_Reset();
		thrust(2) = Z.ADRC_Reset();
	}
	bool log = 0;
	if(log){
		PX4_INFO("z %f %f %f\n\n",(double)thrust(0),(double)thrust(1),(double)thrust(2));
	}
	return thrust;
}

matrix::Vector3f
FaCtrl::torque_update(bool takeoff,const matrix::Quatf &q,float roll,float pitch,float yaw,const float dt)
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
	angle_sp(1) = 0;
	angle_sp(2) = 0;

	if(takeoff){
		torque(0) = Phi.ADRC_Run(angle(0),angle_sp(0),dt,-10,10);
		torque(1) = Theta.ADRC_Run(angle(1),angle_sp(1),dt,-5,5);
		torque(2) = Psai.ADRC_Run(angle(2),angle_sp(2),dt,-5,5);
		Vector3f error = R2D(angle_sp - angle);
		// PX4_INFO("Attit/ude error: %f %f\n\n",(double)angle(0),(double)angle(1));
		PX4_INFO("Attitude error: %f %f %f\n\n",(double)error(0),(double)error(1),(double)error(2));
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

void
FaCtrl::getPositionSetpoint(position_setpoint_onmi_s &pos_setpoint) const
{
	pos_setpoint.x = _pos_sp(0);
	pos_setpoint.y = _pos_sp(1);
	pos_setpoint.z = _pos_sp(2);
}

matrix::Vector3f
FaCtrl::R2D(matrix::Vector3f error)
{
	Vector3f error_d;
	error_d(0) = error(0)*90.f/3.14f;
	error_d(1) = error(1)*90.f/3.14f;
	error_d(2) = error(2)*90.f/3.14f;
	return error_d;
}
