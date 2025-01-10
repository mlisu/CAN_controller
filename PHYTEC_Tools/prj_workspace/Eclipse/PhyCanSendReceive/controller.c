#include "controller.h"

#include <math.h>

#include <stdio.h>

double PIDoutput(double input, double out_ref)
{
	static double const A = (double)KCP*TC/TCI;
	static double const B = (double)KCP*TCD/TC;
	static double err_prev = 0.0;
	static double I = 0.0;
	double i;
	double out;
	double err;

	err = out_ref - input;
	printf("err: %f\t", err);
	printf("I: %f\t", I);
	i = A*err;
	printf("i: %f\t", i);
	I += i;

	out = KCP * err + B * (err - err_prev) + I;
//	printf("out: %f\tI: %f\n", out, I);
	printf("out: %f\n", out);
	err_prev = err;

	if (out >  MAX_OUT)
	{
		I -= i;
		return MAX_OUT;
	}
	if (out < -MAX_OUT)
	{
		I -= i;
		return -MAX_OUT;
	}

	return out;
}

int riddleControl(double input, double out_ref)
{
	static double const A = (double)RKP*TC/RTI/2;
	static double err_prev = 0.0;
	static int out = 0.0;

	double const err = input - out_ref; // inversed
	double const I = A*(err + err_prev);
//	printf("out: %d\n", out);
	out += RKP*(err - err_prev) + I + 0.5; // rounding
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


