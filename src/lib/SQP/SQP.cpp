#include "SQP.hpp"

using namespace matrix;


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

	// float obj_fun(const matrix::Vector<float,6>& x ,matrix::Matrix<float,12,1>x_n){
	// 	matrix::Vector<float,12> X;
	// 		float temp_x;
	// 		float X_norm = 0.f;
	// 		for (int i = 0;i<12;i++)
	// 		{
	// 			for (int j = 0;j <6; j++)
	// 			{
	// 				temp_x += (float)x(j)*v(i,j);
	// 			}
	// 			X(i) = x_n(i,1) +temp_x;
	// 			temp_x = 0.f;
	// 		}
	// 		for (int i = 0;i < 12;i++)
	// 		{
	// 			//X_norm += powf(X(i),2);
	// 		}
	// 		return X_norm;
	// }


// 	//define non-linear inequality constraints c(x)<=0

// 	auto c = [v,x_n](vec& x)->vec {
// 		//vec temp = { 25 * x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3) - 50 * x(3) };
// 		matrix::Vector<float,12> X;
// 		float temp_x;
// 		for (int i = 0;i<12;i++)
// 		{
// 			for (int j = 0;j <6; j++)
// 			{
// 				temp_x += (float)x(j)*v(i,j);
// 			}
// 			X(i) = x_n(i,1) +temp_x;
// 			temp_x = 0.f;
// 		}
// 		float omega[6],alpha[6];
// 		float w_max = 823;
// 		float a_max = M_PI/4;
// 		//PX4_INFO("T = %f \n",(double)_thrust_sp(1));PX4_INFO("A1 = %f \n",(double)A1(4,0));
// 		for (int i = 0;i<6;i++)
// 		{
// 			omega[i] = sqrt(sqrt(powf(X(2*i),2.0)+powf(X(2*i+1),2.0)));
// 			alpha[i] = atan2f(X(2*i),X(2*i+1));
// 		}
// 		vec temp = { {omega[0] - w_max},{omega[1] - w_max},{omega[2] - w_max},
// 				{omega[3] - w_max},{omega[4] - w_max},{omega[5] - w_max},
// 				{abs(alpha[0]) - a_max}};
// 		return temp;
// 	};



// matrix::Vector<float,12>
// ControlAllocator:: optim(matrix::Matrix<float,12,1>x_n)
// {

// 	float v1[12*6] = {-0.5386f,0.3701f,	0.4062f,	0.1537f,	0.1486f,	0.1668f,
// 		-0.0529f,	0.2195f,  	-0.1223f, 	-0.3077f,	0.2112f, 	-0.3595f,
// 		0.1280f,	0.6041f,   	-0.2704f,	0.2978f,	0.1670f,	0.3074f,
// 		0.0457f,   	-0.1381f,	0.0815f,	0.5160f,   	-0.1154f,  	-0.1646f,
// 		0.1727f,	0.5435f,	0.3360f,  	-0.0096f,   	-0.4755f,  	-0.0348f,
// 		0.0606f,   	-0.0922f,	0.0375f,  	-0.2198f,  	-0.1065f,	0.5111f,
// 		0.7902f,	0.1237f,	0.0806f,   	-0.0109f,	0.1367f,  	-0.0225f,
// 		-0.0920f,	0.2576f,   	-0.1224f,  	-0.3004f,	0.1731f,   	-0.3528f,
// 		0.1237f,  	-0.1104f,	0.7572f,  	-0.1550f,	0.1183f,   	-0.1631f,
// 		0.0218f,   	-0.1304f,	0.0482f,	0.5381f,   	-0.0744f,  	-0.1398f,
// 		0.0789f,   	-0.0498f,	0.1508f,	0.1524f,	0.7608f,	0.1791f,
// 		0.0168f,   	-0.1165f,	0.0775f,   	-0.2263f,  	-0.0880f,	0.5056f};
// 	matrix::Matrix<float,12,6> v(v1);
// 	//define a obj_fun
// 	obj_fun f = [v,x_n](vec& x)-> float {
// 		matrix::Vector<float,12> X;
// 		float temp_x;
// 		float X_norm = 0.f;
// 		for (int i = 0;i<12;i++)
// 		{
// 			for (int j = 0;j <6; j++)
// 			{
// 				temp_x += (float)x(j)*v(i,j);
// 			}
// 			X(i) = x_n(i,1) +temp_x;
// 			temp_x = 0.f;
// 		}
// 		for (int i = 0;i<6;i++)
// 		{
// 			X_norm += powf(X(i),2);
// 		}
// 		return X_norm;
// 	};

// 	//define a start point
// 	vec x0 = { 0,0,0,0,0,0 };

// 	/* //optimization without constraints
// 	//using default algorithm BFGS
// 	auto result1 = sci_arma::fmincon(f, x0);

// 	//define linear constraints Ax<=b
// 	mat A = { {1,1,1,0},{2,3,4,5} };
// 	vec b = { 3,15 };

// 	//optimization with inequality linear constraints
// 	auto result2 = sci_arma::fmincon(f, x0, A, b);

// 	//define linear equality constraints
// 	mat Aeq = { {1,0,2,0} };
// 	vec beq = { 3 };

// 	//optimization with mixed linear constraints
// 	auto result3 = sci_arma::fmincon(f, x0, A, b, Aeq, beq);
//  */
// 	//define non-linear inequality constraints c(x)<=0
// 	auto c = [v,x_n](vec& x)->vec {
// 		//vec temp = { 25 * x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3) - 50 * x(3) };
// 		matrix::Vector<float,12> X;
// 		float temp_x;
// 		for (int i = 0;i<12;i++)
// 		{
// 			for (int j = 0;j <6; j++)
// 			{
// 				temp_x += (float)x(j)*v(i,j);
// 			}
// 			X(i) = x_n(i,1) +temp_x;
// 			temp_x = 0.f;
// 		}
// 		float omega[6],alpha[6];
// 		float w_max = 823;
// 		float a_max = M_PI/4;
// 		//PX4_INFO("T = %f \n",(double)_thrust_sp(1));PX4_INFO("A1 = %f \n",(double)A1(4,0));
// 		for (int i = 0;i<6;i++)
// 		{
// 			omega[i] = sqrt(sqrt(powf(X(2*i),2.0)+powf(X(2*i+1),2.0)));
// 			alpha[i] = atan2f(X(2*i),X(2*i+1));
// 		}
// 		vec temp = { {omega[0] - w_max},{omega[1] - w_max},{omega[2] - w_max},
// 				{omega[3] - w_max},{omega[4] - w_max},{omega[5] - w_max},
// 				{abs(alpha[0]) - a_max}};
// 		return temp;
// 	};
// 	vec lb = {-10000,-10000,-10000,-10000,-10000,-10000};
// 	vec ub = {10000,10000,10000,10000,10000,10000};
// 	//optimization with mixed constraints
// 	//set options::algorithm from preset(Powell_modified) to Powell
// 	options opt;
// 	opt.algo = Powell;
// 	auto xfval = sci_arma::fmincon(f, x0, lb, ub, c, opt);

// 	matrix::Vector<float,12> X;float temp_x;
// 	float omega_n[6],alpha_n[6];
// 	for (int i = 0;i<12;i++)
// 	{
// 		for (int j = 0;j <6; j++)
// 		{
// 			temp_x += (float)xfval.x(j)*v(i,j);
// 		}
// 		X(i) = x_n(i,1) +temp_x;
// 		temp_x = 0.f;
// 	}
// 	for(int i = 0;i<6;i++)
// 	{
// 		omega_n[i] = sqrt(sqrt(powf(X(2*i),2.0)+powf(X(2*i+1),2.0)));
// 		alpha_n[i] = atan2f(X(2*i),X(2*i+1));
// 	}
// 	matrix::Vector<float,12>result;
// 	for(int i = 0;i<6;i++)
// 	{
// 		result(i) = omega_n[i];
// 		result(i+6) = alpha_n[i];
// 	}
// 	return result;


// }
