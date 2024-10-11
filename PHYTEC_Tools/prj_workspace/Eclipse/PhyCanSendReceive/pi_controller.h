#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#define OUT_REF 5
#define KCP		10
#define KCI		0.001
#define TC		0.025 //s; controller cycle time


double controllerOutput(double input);

#endif /* PI_CONTROLLER_H_ */
