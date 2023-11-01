/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
// #include <SQP/SQP.hpp>

using namespace matrix;
using namespace time_literals;

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub.advertise();
	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	ScheduleDelayed(50_ms);

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		matrix::Vector<float, NUM_ACTUATORS> actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN:
			tmp = new ActuatorEffectivenessRoverAckermann();
			break;

		case EffectivenessSource::ROVER_DIFFERENTIAL:
			tmp = new ActuatorEffectivenessRoverDifferential();
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessRotors(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Push backup schedule
	ScheduleDelayed(50_ms);

	// Check if parameters have changed
	if (_parameter_update_sub.updated() && !_armed) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 5_ms) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}

	if (!do_update) {
		_last_run = now;

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint);
			_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint);
			c[1](0) = vehicle_torque_setpoint.xyz[0];
			c[1](1) = vehicle_torque_setpoint.xyz[1];
			c[1](2) = vehicle_torque_setpoint.xyz[2];
			c[1](3) = vehicle_thrust_setpoint.xyz[0];
			c[1](4) = vehicle_thrust_setpoint.xyz[1];
			c[1](5) = vehicle_thrust_setpoint.xyz[2];
		}

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			_control_allocation[i]->allocate();
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp);

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}

	} else {
		alloaction_onmi(dt);//全驱动控制分配
	}


	// Publish actuator setpoint and allocator status
	//publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status();
		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::alloaction_onmi(const float dt)
{
	//******************************************************全驱动控制分配*****************************************************************
	//PX4_INFO("Start fully-actuated allocation!!!\n\n");
	////////////////////////////////////////////开始控制分配//////////////////////////////////
	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	//const hrt_abstime now = hrt_absolute_time();
	//const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
	// 更新力和力矩
	torque_sp_s			torque_onmi_sp ;
	thrust_sp_s			thrust_onmi_sp ;
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);//更新遥控器通道值

	// Run allocator on torque changes
	if (_torque_sp_sub.update(&torque_onmi_sp)) {
		_torque_sp_onmi = matrix::Vector3f(torque_onmi_sp.xyz);
		_timestamp_sample = torque_onmi_sp.timestamp_sample;
	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_thrust_sp_sub.update(&thrust_onmi_sp)) {
		_thrust_sp_onmi = matrix::Vector3f(thrust_onmi_sp.xyz);
	}

	float A_inv[72] = {-0.2404, 0.1388, 0.0000, 0.0416, -0.0240, 0.0240,
			   0, 0.0000, 0.1388, 0.8721, 0.5035, 0.5035,
			   0.2404, 0.1388, 0.0000, 0.0416, 0.0240, -0.0240,
			   0, 0.0000, 0.1388, 0.8721, -0.5035, -0.5035,
			   0, -0.2775, 0.0000, -0.0000, 0.0480, 0.0240,
			   0, -0.0000, 0.1388, -0.0000, -1.0070, 0.5035,
			   -0.2404, 0.1388, 0.0000, -0.0416, 0.0240, -0.0240,
			   0, -0.0000, 0.1388, -0.8721, -0.5035, -0.5035,
			   0.2404, 0.1388, -0.0000, -0.0416, -0.0240, 0.0240,
			   0, -0.0000, 0.1388, -0.8721, 0.5035, 0.5035,
			   0, -0.2775, 0.0000, -0.0000, -0.0480, -0.0240,
			   0, 0.0000, 0.1388, -0.0000, 1.0070, -0.5035
			  };
	Matrix<float, 12, 6> A0(A_inv);
	A0.operator *= (100000.0);
	float u[6] = {_manual_control_setpoint.x * 3, _manual_control_setpoint.y * 3,_thrust_sp_onmi(2)+ 5, _torque_sp_onmi(0), _torque_sp_onmi(1), 0};
	Matrix<float, 6, 1> U(u);
	Matrix<float, 12, 1> A1 = A0.operator * (U);
	float omega[6], alpha[6];
	bool optim_flag = 0;
	float omega_max = 813;

	for (int i = 0; i < 6; i++) {
		omega[i] = sqrt(sqrt(powf(A1(2 * i, 0), 2.0) + powf(A1(2 * i + 1, 0), 2.0)));
		alpha[i] = atan2f(A1(2 * i, 0), A1(2 * i + 1, 0));

		if (omega[i] > omega_max) {optim_flag = 1;}
	}

	//PX4_INFO("U = %f %f %f %f %f %f\n\n",(double)U(0,0),(double)U(1,0),(double)U(2,0),(double)U(3,0),(double)U(4,0),(double)U(5,0));
	act_optim_data.flag = optim_flag;

	//PX4_INFO("%f  %f  %f %f %f %f\n\n",(double)act_optim_data.xn[0],(double)act_optim_data.xn[1],(double)act_optim_data.xn[2]
	//,(double)act_optim_data.xn[3],(double)act_optim_data.xn[4],(double)act_optim_data.xn[5]);
	for (int i = 0; i < 12; i++) {
		act_optim_data.controls[i] = A1(i, 0);
	}

	//PX4_INFO("%f  %f  %f\n\n",(double)act_optim_data.xn[0],(double)act_optim_data.xn[1],(double)act_optim_data.xn[2]);
	//_act_optim_pub.publish(act_optim_data);

	/////////////////////////////计算舵机输出///////////////////////////////////////////
	float alpha_d[6], alpha_set[6];
	MRS mrs;

	for (int i = 0; i < 6; i++) {
		alpha_d[i] = (alpha[i] * 180) / (float)M_PI;
		alpha_set[i] = (alpha_d[i] / 90) * 1 + 0;

		if (i == 3) {alpha_set[i] = -alpha_set[i];} //0/2/4号舵机反方向
	}

	// mrs.Mrs_update(alpha_set,dt).copyTo(alpha_set);//更新MRS模型输出
	/////////////////////////////计算电机输出///////////////////////////////////////////
	float omega_set[6];

	for (int i = 0; i < 6; i++) {
		omega_set[i] = (omega[i] / 1160.544f) + 0 < 0.9f ? (omega[i] / 1160.544f) + 0 : 0.9f;
		if(alpha_set[i] > 1 || alpha_set[i]<-1)
		{
			alpha_set[i] = 0;
			omega_set[i] = 0;
		}
		actuator_motors.control[i] = omega_set[i];
		actuator_servos.control[i] = alpha_set[i];
	}

	actuator_motors.control[6] = 0; actuator_motors.control[7] = 0;
	actuator_servos.control[6] = 0; actuator_servos.control[7] = 0;

	////////////////////////////////电机转速优化///////////////////////////////////////////
	_act_optim_pub.publish(act_optim_data);//发布优化信息

	/////////////////////////////发布电机和舵机输出///////////////////////////////////////////
	_actuator_servos_pub.publish(actuator_servos);// 发布舵机信号;
	_actuator_motors_pub.publish(actuator_motors);// 输出电机信号;

	publish_anti_windup(windup(alpha_set, omega_set)); //获取anti-windup，并发布消息；
	bool log = 1;

	if (log) {
		PX4_INFO("U:%f %f %f %f %f  %f\n\n", (double)u[0], (double)u[1], (double)u[2], (double)u[3], (double)u[4],
			 (double)u[5]);
		PX4_INFO("w:%f %f %f %f %f %f\n\n", (double)actuator_motors.control[0], (double)actuator_motors.control[1],
			 (double)actuator_motors.control[2],
			 (double)actuator_motors.control[3], (double)actuator_motors.control[4], (double)actuator_motors.control[5]);
		PX4_INFO("a:%f %f %f %f %f %f\n\n", (double)actuator_servos.control[0], (double)actuator_servos.control[1],
			 (double)actuator_servos.control[2],
			 (double)actuator_servos.control[3], (double)actuator_servos.control[4], (double)actuator_servos.control[5]);
	}
}

matrix::Vector<float, 6> ControlAllocator::windup(const float *alphad, const float *omega)
{
	float Kt = 1.201e-5;
	float Kq = 1.57e-7;
	float l  = 0.275;
	Vector<float, 6>alpha;

	for (int i = 0; i < 6; i++) {alpha(i) = alphad[i];}

	float A[6 * 6] = {-(sqrtf(3) / 2) *Kt * sinf(alpha(1)), (sqrtf(3) / 2) *Kt * sinf(alpha(2)),  0,
			  -(sqrtf(3) / 2) *Kt * sinf(alpha(4)), (sqrtf(3) / 2) *Kt * sinf(alpha(5)),  0,

			  0.5f * Kt * sinf(alpha(1)),              0.5f * Kt * sinf(alpha(2)),         -Kt * sinf(alpha(3)),
			  0.5f * Kt * sinf(alpha(4)),             0.5f * Kt * sinf(alpha(5)),        -Kt * sinf(alpha(6)),

			  Kt * cosf(alpha(1)),  Kt * cosf(alpha(2)), Kt * cosf(alpha(3)), Kt * cosf(alpha(4)), Kt * cosf(alpha(5)), Kt * cosf(alpha(6)),

			  (sqrtf(3) / 2) *(l *Kt * cosf(alpha(1)) + Kq * sinf(alpha(1))), (sqrtf(3) / 2) *(l *Kt * cosf(alpha(2)) + Kq * sinf(alpha(2))), 0,
			  -(sqrtf(3) / 2) *(l *Kt * cosf(alpha(4)) + Kq * sinf(alpha(4))), -(sqrtf(3) / 2) *(l *Kt * cosf(alpha(5)) + Kq * sinf(alpha(5))), 0,

			  -0.5f * (-l *Kt * cosf(alpha(1)) + Kq * sinf(alpha(1))), 0.5f * (-l *Kt * cosf(alpha(2)) + Kq * sinf(alpha(2))), (-l *Kt * cosf(alpha(3)) + Kq * sinf(alpha(3))),
			  0.5f * (-l *Kt * cosf(alpha(4)) + Kq * sinf(alpha(4))), -0.5f * (-l *Kt * cosf(alpha(5)) + Kq * sinf(alpha(5))), -(-l *Kt * cosf(alpha(6)) + Kq * sinf(alpha(6))),

			  (l *Kt * cosf(alpha(1)) + Kq * sinf(alpha(1))), -(l *Kt * cosf(alpha(2)) + Kq * sinf(alpha(2))), (l *Kt * cosf(alpha(3)) + Kq * sinf(alpha(3))),
			  -(l *Kt * cosf(alpha(4)) + Kq * sinf(alpha(4))), (l *Kt * cosf(alpha(5)) + Kq * sinf(alpha(5))), -(l *Kt * cosf(alpha(6)) + Kq * sinf(alpha(6)))
			 };
	matrix::Matrix<float, 6, 6>matrixA(A); //定义控制分配矩阵
	matrix::Matrix<float, 6, 1>W(omega); //定义转速矩阵
	matrix::Matrix<float, 6, 1>W2;

	for (int i = 0; i < 6; i++) {W2(i, 0) = powf(W(i, 0), 2);} //转速矩阵平方

	matrix::Matrix<float, 6, 1>U = matrixA.operator * (W2);
	return U;
}
void ControlAllocator::publish_anti_windup(matrix::Vector<float, 6> uast)
{
	antiwindup.timestamp = hrt_absolute_time();
	uast.copyTo(antiwindup.usat);
	_anti_windup_pub.publish(antiwindup);
}


/* matrix::Vector<float,12>
ControlAllocator:: optim(matrix::Matrix<float,12,1>x_n)
{

	float v1[12*6] = {-0.5386f,0.3701f,	0.4062f,	0.1537f,	0.1486f,	0.1668f,
		-0.0529f,	0.2195f,  	-0.1223f, 	-0.3077f,	0.2112f, 	-0.3595f,
		0.1280f,	0.6041f,   	-0.2704f,	0.2978f,	0.1670f,	0.3074f,
		0.0457f,   	-0.1381f,	0.0815f,	0.5160f,   	-0.1154f,  	-0.1646f,
		0.1727f,	0.5435f,	0.3360f,  	-0.0096f,   	-0.4755f,  	-0.0348f,
		0.0606f,   	-0.0922f,	0.0375f,  	-0.2198f,  	-0.1065f,	0.5111f,
		0.7902f,	0.1237f,	0.0806f,   	-0.0109f,	0.1367f,  	-0.0225f,
		-0.0920f,	0.2576f,   	-0.1224f,  	-0.3004f,	0.1731f,   	-0.3528f,
		0.1237f,  	-0.1104f,	0.7572f,  	-0.1550f,	0.1183f,   	-0.1631f,
		0.0218f,   	-0.1304f,	0.0482f,	0.5381f,   	-0.0744f,  	-0.1398f,
		0.0789f,   	-0.0498f,	0.1508f,	0.1524f,	0.7608f,	0.1791f,
		0.0168f,   	-0.1165f,	0.0775f,   	-0.2263f,  	-0.0880f,	0.5056f};
	matrix::Matrix<float,12,6> v(v1);
	//define a obj_fun
	obj_fun f = [v,x_n](vec& x)-> float {
		matrix::Vector<float,12> X;
		float temp_x;
		float X_norm = 0.f;
		for (int i = 0;i<12;i++)
		{
			for (int j = 0;j <6; j++)
			{
				temp_x += (float)x(j)*v(i,j);
			}
			X(i) = x_n(i,1) +temp_x;
			temp_x = 0.f;
		}
		for (int i = 0;i<6;i++)
		{
			X_norm += powf(X(i),2);
		}
		return X_norm;
	};

	//define a start point
	vec x0 = { 0,0,0,0,0,0 };

	//define non-linear inequality constraints c(x)<=0
	auto c = [v,x_n](vec& x)->vec {
		//vec temp = { 25 * x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3) - 50 * x(3) };
		matrix::Vector<float,12> X;
		float temp_x;
		for (int i = 0;i<12;i++)
		{
			for (int j = 0;j <6; j++)
			{
				temp_x += (float)x(j)*v(i,j);
			}
			X(i) = x_n(i,1) +temp_x;
			temp_x = 0.f;
		}
		float omega[6],alpha[6];
		float w_max = 823;
		float a_max = M_PI/4;
		//PX4_INFO("T = %f \n",(double)_thrust_sp(1));PX4_INFO("A1 = %f \n",(double)A1(4,0));
		for (int i = 0;i<6;i++)
		{
			omega[i] = sqrt(sqrt(powf(X(2*i),2.0)+powf(X(2*i+1),2.0)));
			alpha[i] = atan2f(X(2*i),X(2*i+1));
		}
		vec temp = { {omega[0] - w_max},{omega[1] - w_max},{omega[2] - w_max},
				{omega[3] - w_max},{omega[4] - w_max},{omega[5] - w_max},
				{abs(alpha[0]) - a_max}};
		return temp;
	};
	vec lb = {-10000,-10000,-10000,-10000,-10000,-10000};
	vec ub = {10000,10000,10000,10000,10000,10000};
	//optimization with mixed constraints
	//set options::algorithm from preset(Powell_modified) to Powell
	options opt;
	opt.algo = Powell;
	auto xfval = sci_arma::fmincon(f, x0, lb, ub, c, opt);

	matrix::Vector<float,12> X;float temp_x;
	float omega_n[6],alpha_n[6];
	for (int i = 0;i<12;i++)
	{
		for (int j = 0;j <6; j++)
		{
			temp_x += (float)xfval.x(j)*v(i,j);
		}
		X(i) = x_n(i,1) +temp_x;
		temp_x = 0.f;
	}
	for(int i = 0;i<6;i++)
	{
		omega_n[i] = sqrt(sqrt(powf(X(2*i),2.0)+powf(X(2*i+1),2.0)));
		alpha_n[i] = atan2f(X(2*i),X(2*i+1));
	}
	matrix::Vector<float,12>result;
	for(int i = 0;i<6;i++)
	{
		result(i) = omega_n[i];
		result(i+6) = alpha_n[i];
	}
	return result;

} */

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}

void
ControlAllocator::publish_control_allocator_status()
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// TODO: handle multiple matrices & disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[0]->getAllocatedControl();
	control_allocator_status.allocated_torque[0] = allocated_control(0);
	control_allocator_status.allocated_torque[1] = allocated_control(1);
	control_allocator_status.allocated_torque[2] = allocated_control(2);
	control_allocator_status.allocated_thrust[0] = allocated_control(3);
	control_allocator_status.allocated_thrust[1] = allocated_control(4);
	control_allocator_status.allocated_thrust[2] = allocated_control(5);

	// Unallocated control
	matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[0]->getControlSetpoint() - allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(unallocated_control(0), unallocated_control(1),
			unallocated_control(2)).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(unallocated_control(3), unallocated_control(4),
			unallocated_control(5)).norm_squared() < 1e-6f);

	// Actuator saturation
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp = _control_allocation[0]->getActuatorSetpoint();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min = _control_allocation[0]->getActuatorMin();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max = _control_allocation[0]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	_control_allocator_status_pub.publish(control_allocator_status);
}

void
ControlAllocator::publish_actuator_controls()
{
	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	uint32_t stopped_motors = _actuator_effectiveness->getStoppedMotors();

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;

		if (stopped_motors & (1u << motors_idx)) {
			actuator_motors.control[motors_idx] = NAN;
		}

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		int servos_idx;

		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
			int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

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

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness.T =");
		effectiveness.T().print();
		PX4_INFO("  minimum =");
		_control_allocation[i]->getActuatorMin().T().print();
		PX4_INFO("  maximum =");
		_control_allocation[i]->getActuatorMax().T().print();
		PX4_INFO("  Configured actuators: %i", _control_allocation[i]->numConfiguredActuators());
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(MODULE_NAME, "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
