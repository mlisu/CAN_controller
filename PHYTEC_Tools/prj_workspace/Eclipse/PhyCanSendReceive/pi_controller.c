#include "pi_controller.h"

double controllerOutput(double input)
{
	static double err_prev = 0;
	static double integral = 0;

	double const err = OUT_REF - input;

	integral += KI* TS/2*(err + err_prev);

	err_prev = err;

	return KP*err + integral;
}


