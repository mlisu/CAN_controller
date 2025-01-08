#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// common params:
#define TC		0.01		//s; controller cycle time

// params for suspension control (PID):
#define KCP		40000	// gain
#define TCI		4		// integral time constant
#define TCD		0.25	// derivative time constant
#define MAX_OUT 2000	// N
#define OUT_REF 0

// riddle controller params
#define SAMPLS   21		// samples number to compute RMS
#define RKP		 1000	// gain
#define RTI		 0.1	// integral time constant
#define RMAX_OUT 5000	// Ns/m - max dumpers coefficient
#define ROUT_REF 20		// m/s^2 - reference RMS


double controllerOutput(double input, double out_ref);
double PIDoutput(double input, double out_ref);
double PIDoutputTustin(double input, double out_ref);
int riddleControl(double input, double out_ref);

double computeRMS(double acc_front, double acc_rear);

#endif /* CONTROLLER_H_ */
