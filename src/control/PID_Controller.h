#pragma once

#include <common/base_classes/Controller.h>
#include <common/pid.h>
#include <common/foc_utils.h>

class PID_Controller : public Controller, public PIDController
{
public:
	PID_Controller(float P, float I, float D, float Ramp = NOT_SET, float Limit = NOT_SET);

	//Input: Error
	//Output: Command
	virtual float Control(float input, float dt) override;
};