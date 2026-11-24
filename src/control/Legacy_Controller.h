#pragma once

#include "Legacy_Current_Controller.h"
#include "PID_Controller.h"
#include "LPF_Controller.h"
#include <common/base_classes/FOCMotor.h>

class Legacy_Controller : public Legacy_Current_Controller
{
public:
	float target = 0.0f;
	
	MotionControlType controller; //!< parameter determining the control loop to be used

	// controllers and low pass filters
	PID_Controller PID_velocity;//!< parameter determining the velocity PID configuration
	PID_Controller P_angle;	//!< parameter determining the position PID configuration 
	LPF_Controller LPF_velocity;//!<  parameter determining the velocity Low pass filter configuration 
	LPF_Controller LPF_angle;//!<  parameter determining the angle low pass filter configuration 
	unsigned int motion_downsample = DEF_MOTION_DOWNSMAPLE; //!< parameter defining the ratio of downsampling for move commad
	unsigned int motion_cnt = 0; //!< counting variable for downsampling for move commad

	Legacy_Controller();

	//Set target in monitor pointers (index 6)
	virtual void linkMotor(FOCMotor *in_motor = nullptr) override;

	//Input : target position
	//Output: 0 if ok, NOT_SET if error
	virtual float Control(float input, float dt) override;
	virtual void Command(parsed_user_command command, Commander &source) override;


	void monitor();
};
