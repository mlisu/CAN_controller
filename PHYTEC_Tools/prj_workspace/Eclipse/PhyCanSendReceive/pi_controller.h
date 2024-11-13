#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#define OUT_REF 5
#define KCP		10
#define KCI		0.001
#define TC		0.3		//s; controller cycle time
#define TCI		1		// integral time constant


double controllerOutput(double input, double out_ref);

#endif /* PI_CONTROLLER_H_ */
