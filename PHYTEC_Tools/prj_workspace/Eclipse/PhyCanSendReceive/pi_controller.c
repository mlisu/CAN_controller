#include "pi_controller.h"

#include <math.h>

#include <stdio.h>

double controllerOutput(double input, double out_ref)
{
	static double err_prev = 0;
	static double integral = 0;

	double out;
	double const err = out_ref - input;

	integral += TC/TCI/2*(err + err_prev);

	err_prev = err;

	out = KCP*(err + integral);

	if (out >  MAX_OUT) return  MAX_OUT;
	if (out < -MAX_OUT) return -MAX_OUT;

	return out;
}

double PIDoutput(double input, double out_ref)
{
	static double const A = (double)KCP*TC/TCI;
	static double const B = (double)KCP*TCD/TC;
	static double out = 0.0;
	static double err[3] = {0.0, 0.0, 0.0};
	double I;

	err[0] = out_ref - input;
	I = A*err[0];

	out += KCP * (err[0] - err[1]) + B * (err[0] - 2*err[1] + err[2]) + I;

	err[2] = err[1];
	err[1] = err[0];

	if (out >  MAX_OUT)
	{
		out -= I;
		return MAX_OUT;
	}
	if (out < -MAX_OUT)
	{
		out -= I;
		return -MAX_OUT;
	}

	return out;
}

double PIDoutputTustin(double input, double out_ref)
{
	static double const A = (double)KCP*TC/2/TCI;
	static double const B = (double)KCP*2*TCD/TC;
	static double out[3] = {0.0, 0.0, 0.0}; // is used also as out_prev_prev
	static double err[3] = {0.0, 0.0, 0.0};
	double I;

	err[0] = out_ref - input;
	I = A * (err[0] + 2*err[1] + err[2]);

	out[0] = out[2] + KCP *(err[0] - err[2]) + I + 		// P + I
			 	 	  B * (err[0] - 2*err[1] + err[2]); // D

	out[2] = out[1];
	out[1] = out[0];

	err[2] = err[1];
	err[1] = err[0];

	if (out[0] > MAX_OUT)
	{
		out[1] -= I;
		return MAX_OUT;
	}
	if (out[0] < -MAX_OUT)
	{
		out[1] -= I;
		return -MAX_OUT;
	}

	return out[0];
}

// TODO make generic fn for PI suspension and riddle by taking 3rd arg of struct with PI params
int riddleControl(double input, double out_ref)
{
	static double const A = (double)RKP*TC/RTI/2;
	static double err_prev = 0.0;
	static int out = 0.0;

	double const err = input - out_ref; // inversed
	double const I = A*(err + err_prev);

	out += RKP*(err - err_prev) + I + 0.5;
	err_prev = err;

	if (out > RMAX_OUT)
	{
		out -= I;
		return  RMAX_OUT;
	}
	if (out < 0)
	{
		out -= I;
		return 0;
	}

	return out;
}

double computeRMS(double acc_front, double acc_rear)
{
	static int insert_idx = 0;
	static double rmss[SAMPLS] = {0.0};
	static unsigned char flag = 0;
	static int in_buf = 0;
	static double sum = 0.0;

	double const acc = (acc_front + acc_rear) / 2;

	if(!flag)
	{
		in_buf = insert_idx + 1;
		if(in_buf == SAMPLS) flag = 1;
	}

	// adding all array elements can be more accurate (we skip one float operation -> "-=")
	sum -= rmss[insert_idx];
	rmss[insert_idx] = acc*acc;
	sum += rmss[insert_idx];

	if (++insert_idx == SAMPLS)
	{
		insert_idx = 0;
	}
//	printf("sum: %f\trmss[insert_idx]: %f\tsqrt: %f\n", sum, rmss[insert_idx], sqrt(sum / in_buf));

	return sqrt(sum / in_buf);
}


