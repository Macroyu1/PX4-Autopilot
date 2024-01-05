#pragma once


#include <stdint.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

#define ABS(X)  (((X)>0)?(X):-(X))

class MRS
{
public:
	MRS() = default;
	~MRS() = default;

	//matrix::Vector<float,6>Mrs_update(matrix::Vector<float,6> , const float dif_time);

	matrix::Vector<float,6> Mrs_update(float *object,const float dif_time)
	{
		Setdiftime(dif_time);
		for(int i = 0;i < 6;i++)
		{
			alpha_d(i) = (object[i] - alpha(i))/dt;
			lamda(i) = alpha_d(i) + K*(object[i] - sigma(i)) + Kr*(lamda(i) - sigma_d(i));
			Saturation();
			sigma(i) += sigma_d(i)*dt;
			alpha(i) += sigma(i);
		}
		return alpha;
	};
private:
	void Setdiftime(const float dif_time){dt = dif_time;};
	void Saturation(){for(int i = 0;i < 6;i++){sigma_d(i) = (ABS(lamda(i)) < alpha_d_max/dt)? lamda(i) : (lamda(i)/ABS(lamda(i)))*alpha_d_max/dt;}}

	float dt;//sample time

	float alpha_d_max = 2.05f/0.7f;
	float K = 0.1;
	float Km = 0.15;
	float Kr = 0.2;

	matrix::Vector<float,6> alpha;
	matrix::Vector<float,6> alpha_d;
	matrix::Vector<float,6> lamda;
	matrix::Vector<float,6> sigma;
	matrix::Vector<float,6> sigma_d;



};
