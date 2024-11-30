#include "pi_controller.h"

double controllerOutput(double input, double out_ref)
{
	static double err_prev = 0;
	static double integral = 0;

	double const err = out_ref - input;

	integral += KCI* TC/TCI/2*(err + err_prev);

	err_prev = err;

	return KCP*(err + integral);
}


