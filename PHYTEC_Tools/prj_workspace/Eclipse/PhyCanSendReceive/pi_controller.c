#include "pi_controller.h"

#include <math.h>

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
	static double out = 0.0;
	static double err[3] = {0.0, 0.0, 0.0};

	err[0] = out_ref - input;

	out += KCP * ( (1 + TC/TCI + TCD/TC)*err[0] - (1 + 2*TCD/TC)*err[1] + TCD/TC*err[2] );
	err[2] = err[1];
	err[1] = err[0];

	if (out >  MAX_OUT) return  MAX_OUT;
	if (out < -MAX_OUT) return -MAX_OUT;

	return out;
}

double riddleControl(double input, double out_ref)
{
	static double err_prev = 0;
	static double integral = 0;

	double out;
	double const err = out_ref - input;

	integral += TC/TCI/2*(err + err_prev);

	err_prev = err;

	out = KCP*(err + integral);

	if (out > RMAX_OUT) return  RMAX_OUT;
	if (out < 0) return 0;

	return out;
}

double computeRMS(double acc_front, double acc_rear)
{
	static long long insert_idx = 0;
	static double rmss[SAMPLS];
	static unsigned char flag = 0;
	static int in_buf = 0;

	double const acc = (acc_front + acc_rear) / 2;
	double sum = 0;
	int i;

	if(!flag)
	{
		if(in_buf == SAMPLS) flag = 1;
		else in_buf = insert_idx + 1;
	}

	rmss[insert_idx++ % SAMPLS] = acc*acc;

	for(i = 0; i < in_buf; i++)
	{
		sum += rmss[i];
	}

	return sqrt(sum / in_buf);
}


