/**
 * @file adrc.h
 *
 * Definition of generic ADRC controller.
 *
 * @author Sunyi
 */

#ifndef LADRC_H_
#define LADRC_H_
#define ABS(X)  (((X)>0)?(X):-(X))
#include <stdint.h>

__BEGIN_DECLS

typedef   signed short     int int16;
typedef unsigned short     int uint16;

typedef struct
{
/*****扩张状态观测器*******/
/******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
float z1;
float z2;
float z3;//根据控制对象输入与输出，提取的扰动信息
float state_e;
float w0;


/**********系统状态误差反馈率*********/
float wc;//
float u0 = 0;//非线性组合系统输出
float u;//带扰动补偿后的输出
float b0;//扰动补偿
}Parm;

__EXPORT void ADRC_Init(Parm *state,const float wc,const float b0);
__EXPORT void ADRC_Reset(Parm *state);
__EXPORT float ADRC_Saturation(const float u,const float u_min,const float u_max);
__EXPORT float ADRC_Control(Parm *state,const float state_feedback,const float state_d,const float dt);
//extern Fhan_Data ADRC_Pitch_Controller,ADRC_Roll_Controller;
__END_DECLS
#endif /* PID_H_ */
