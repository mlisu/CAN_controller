#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#define KCP		40000
//#define KCI		0.001
#define TC		0.01		//s; controller cycle time
#define TCI		4		// integral time constant
#define MAX_OUT 2000	// N

// PID params - only derivative part, PI is the same
#define TCD		0.25
// riddle controller params
#define RMS		 20 		// m/s^2 - reference RMS
#define SAMPLS   21		// samples number to compute RMS
#define RP		 50		// gain for riddle control
#define RTI		 4		// integral time constant for riddle control
#define RMAX_OUT 1000	// Ns/m - max dumpers coefficient




double controllerOutput(double input, double out_ref);
double PIDoutput(double input, double out_ref);
double riddleControl(double input, double out_ref);

double computeRMS(double acc_front, double acc_rear);

#endif /* PI_CONTROLLER_H_ */
