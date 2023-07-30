/**
 * @file adrc.c
 *
 * Implementation of generic ADRC controller.
 *
 * @author sunyi
 */

#include "ladrc.h"
#include <math.h>
#include <px4_platform_common/defines.h>


float ADRC_Saturation(const float u,const float u_min,const float u_max){
  return ((u)<(u_min)?(u_min):((u)>(u_max)?(u_max):(u)));
}


void ADRC_Init(Parm *state,const float wc,const float b0)
{
  //**************初始化参数******************
  state->b0 = b0;
  state->wc = wc;
  if (b0 > 2){
    state->w0 = 15*wc;
  }else{
    state->w0 = 5*wc;
  }
}
void ADRC_Reset(Parm *state)
{
  state->z1 = 0;
  state->z2 = 0;
  state->z3 = 0;
  state->u0 = 0;
  state->u  = 0;
}

float ADRC_Control(Parm *state,const float state_feedback,const float state_d,const float dt)
{
  /************扩张状态观测器********************/
  float beta01 = 3*state->w0;
  float beta02 = 3*pow(state->w0,2);
  float beta03 = pow(state->w0,3);
  state->state_e = state->z1 - state_feedback;
  state->z1 = state->z1 + dt*(state->z2 - beta01*state->state_e);
  state->z2 = state->z2 + dt*(state->z3 - beta02*state->state_e + state->b0*state->u0);
  state->z3 = state->z3 - dt*beta03*state->state_e;



  //PX4_INFO("%f %f %f\n",(double)state->z1,(double)state->z2,(double)state->z3);
  /************状态误差反馈控制律********************/
  float kp = pow(state->wc,2);
  float kd = 2*state->wc;
  state->u = kp*(state_d-state->z1)-kd*state->z2-state->z3;//
  return state->u0 = state->u/state->b0;
}
