#pragma once

#include <common/base_classes/Controller.h>
#include <common/lowpass_filter.h>

class LPF_Controller : public Controller, public LowPassFilter
{
public:
	LPF_Controller(float Tf);

	//Input: value
	//Output: filtered value
	virtual float Control(float input, float dt) override;
};