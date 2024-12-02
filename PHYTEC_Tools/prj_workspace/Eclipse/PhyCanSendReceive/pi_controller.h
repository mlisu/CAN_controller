#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#define OUT_REF 5
#define KCP		40000
#define KCI		0.001
#define TC		0.01		//s; controller cycle time
#define TCI		4		// integral time constant
#define MAX_OUT 2000	// N

// PID params - only derivative part, PI is the same
#define TCD		0.25

#define

double controllerOutput(double input, double out_ref);

#endif /* PI_CONTROLLER_H_ */
