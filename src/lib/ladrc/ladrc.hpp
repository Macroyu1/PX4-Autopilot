/**
 * @file ladrc.hpp
 *
 * Definition of generic ADRC controller.
 *
 * @author Sunyi
 */
#pragma once

#include "ladrc.h"
#include <px4_platform_common/log.h>
#define ABS(X) (((X) > 0) ? (X) : -(X))


class LADRC
{
private:
	float z1;
	float z2;
	float z3;
	float state_e;
	float state_feedback;
	float state_setpoint;
	float w0;
	float wc;
	float u0;
	float u;
	float b0;
	float h;
	float u_max;
	float u_min;

	/************扩张状态观测器********************/
	void LESO()
	{
		float beta01 = 3 * this->w0;
		float beta02 = 3 * this->w0 * this->w0;
		float beta03 = this->w0 * this->w0 * this->w0;

		this->state_e = this->z1 - this->state_feedback;
		this->z1 += this->h * (this->z2 - beta01 * this->state_e);
		this->z2 += this->h * (this->z3 - beta02 * this->state_e + this->b0 * this->u0);
		this->z3 += - this->h * beta03 * this->state_e;
	}

	void MRS()
	{

	}
	/************状态误差反馈控制律********************/
	void LSEF()
	{
		float kp = this->wc * this->wc;
		float kd = 2 * this->wc;
		this->u = kp * (this->state_setpoint - this->z1) -kd * this->z2 - this->z3;//
		this->u0 = (this->u)/this->b0;
	}

	float Saturation()
	{
		return (this->u0 < this->u_min) ? this->u_min : (this->u0 > this->u_max) ? this->u_max : this->u0;
	}

	bool isvalid(const float x)
	{
		if (x > 0 || x < 0.1f) //if x != NAN
		{
			return 1;
		}else{
			return 0;
		}

	}

public:
	//**************初始化参数******************
	LADRC(const float wc_in,const float b0_in){
		this->wc = wc_in;
		this->b0 = b0_in;
		this->z1 = 0;
		this->z2 = 0;
		this->z3 = 0;
		this->u  = 0;
		this->u0 = 0;
	}
	LADRC(const float wc_in,const float b0_in,const float c20_in);
	~LADRC() = default;

	void ADRC_Log(bool eso)
	{
		PX4_INFO("input = %f feedback = %f estimate = %f output = %f\n\n",
			(double)this->state_setpoint,(double)this->state_feedback,(double)this->z1,(double)this->u0);
		if(eso){PX4_INFO("z1 = %f z2 = %f z3 = %f \n\n",(double)this->z1,(double)this->z2,(double)this->z3);}
	}

	float ADRC_Reset()
	{
		this->z1 = 0;
		this->z2 = 0;
		this->z3 = 0;

		return this->u;
	}

	float ADRC_Run(const float state_feedback_in,const float state_setpoint_in,const float dt,const float min,const float max)
	{

		this->state_setpoint = isvalid(state_setpoint_in) ? state_setpoint_in : isvalid(this->state_setpoint)?this->state_setpoint:0;
		this->state_feedback = isvalid(state_feedback_in) ? state_feedback_in : this->state_setpoint;
		this->h              = dt;
		this->u_min          = min;
		this->u_max          = max;
		LESO();
		LSEF();
		return Saturation();
	}

	float ADRC_XY(const float state_feedback_in,const float state_setpoint_in,const float state_feedback_dot,const float dt,const float min,const float max)
	{
		this->state_setpoint = isvalid(state_setpoint_in) ? state_setpoint_in : isvalid(this->state_setpoint)?this->state_setpoint:0;
		this->state_feedback = isvalid(state_feedback_in) ? state_feedback_in : this->state_setpoint;
		// this->state_feedback = state_feedback_in;
		// this->state_setpoint = state_setpoint_in;
		this->h              = dt;
		this->u_min          = min;
		this->u_max          = max;
		LESO();

		float kp = this->wc * this->wc;
		float kd = 2 * this->wc;
		this->u = kp * (this->state_setpoint - this->z1)  + kd * (state_feedback_dot - this->z2)- this->z3;//
		this->u0 = (this->u)/this->b0;

		return Saturation();
	}

	float ADRC_Z(const float state_feedback_in,const float state_setpoint_in,const float state_feedback_dot, float dt,const float min,const float max)
	{

		this->state_setpoint = isvalid(state_setpoint_in) ? state_setpoint_in : isvalid(this->state_setpoint)?this->state_setpoint:0;
		this->state_feedback = isvalid(state_feedback_in) ? state_feedback_in : this->state_setpoint;
		// this->state_feedback = state_feedback_in;
		// this->state_setpoint = state_setpoint_in;
		this->h              = dt;
		this->u_min          = min;
		this->u_max          = max;
		LESO();

		float kp = this->wc * this->wc;
		float kd = 2 * this->wc;
		this->u = kp * (this->state_setpoint - this->z1)  + kd * (state_feedback_dot - this->z2)- this->z3;//
		this->u0 = this->u / this->b0;

		return Saturation();
	}
};
