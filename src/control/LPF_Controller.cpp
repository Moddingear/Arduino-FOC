#include "LPF_Controller.h"

LPF_Controller::LPF_Controller(float Tf)
	:LowPassFilter(Tf)
{
}

float LPF_Controller::Control(float input, float dt)
{
	return operator()(input, dt);
}