#include "pi_controller.h"

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


