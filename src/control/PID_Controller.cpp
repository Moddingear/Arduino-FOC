#include "PID_Controller.h"

PID_Controller::PID_Controller(float P, float I, float D, float Ramp, float Limit)
	:PIDController(P, I, D, Ramp, Limit)
{

}

float PID_Controller::Control(float input, float dt)
{
	return operator()(input, dt);
}