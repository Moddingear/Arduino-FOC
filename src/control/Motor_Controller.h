#pragma once

#include <common/base_classes/Controller.h>

class FOCMotor;

//Controller with a motor linked to it
class Motor_Controller : public Controller
{
public:
	FOCMotor *motor;

	Motor_Controller();

	virtual void linkMotor(FOCMotor *in_motor = nullptr);
};


