#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#define KCP		40000
//#define KCI		0.001
#define TC		0.01		//s; controller cycle time
#define TCI		4		// integral time constant
#define MAX_OUT 2000	// N
#define OUT_REF 0

// PID params - only derivative part, PI is the same
#define TCD		0.25
// riddle controller params
#define RMS		 20 		// m/s^2 - reference RMS
#define SAMPLS   21		// samples number to compute RMS
#define RKP		 1000		// gain for riddle control
#define RTI		 0.1		// integral time constant for riddle control
#define RMAX_OUT 3000	// Ns/m - max dumpers coefficient
#define ROUT_REF 20




double controllerOutput(double input, double out_ref);
double PIDoutput(double input, double out_ref);
int riddleControl(double input, double out_ref);

double computeRMS(double acc_front, double acc_rear);

#endif /* PI_CONTROLLER_H_ */
