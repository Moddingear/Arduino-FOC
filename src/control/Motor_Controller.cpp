#include "Motor_Controller.h"
#include <common/base_classes/FOCMotor.h>

Motor_Controller::Motor_Controller()
	:motor(nullptr)
{
}

void Motor_Controller::linkMotor(FOCMotor *in_motor)
{
	if (motor)
	{
		#if SIMPLEFOC_SPLIT_CONTROLLER
		if (motor->controller == this)
		{
			motor->controller = nullptr;
		}
		#endif
	}
	if (in_motor)
	{
		if (in_motor != nullptr)
		{
			//uh oh
		}
		#if SIMPLEFOC_SPLIT_CONTROLLER
		in_motor->controller = this;
		#endif
		motor = in_motor;
	}
}