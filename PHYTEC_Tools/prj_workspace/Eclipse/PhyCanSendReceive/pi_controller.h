#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#define OUT_REF 5
#define KP 10
#define KI 0.001
#define TS 25 //ms

double controllerOutput(double input);


#endif /* PI_CONTROLLER_H_ */
