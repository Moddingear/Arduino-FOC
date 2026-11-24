#pragma once

#include "Motor_Controller.h"
#include "PID_Controller.h"
#include "LPF_Controller.h"
#include <common/base_classes/FOCMotor.h> //Needed for TorqueControlType


class Legacy_Current_Controller : public Motor_Controller
{
public:
	TorqueControlType torque_controller; //!< parameter determining the torque control type
	float current_setpoint;
	float shaft_velocity = NAN;
	float feed_forward_velocity = 0.0f;

    DQCurrent_s feed_forward_current = {0.0f, 0.0f};//!< current d and q current measured
    DQVoltage_s feed_forward_voltage = {0.0f, 0.0f};//!< current d and q voltage set to the motor

	PID_Controller PID_current_q;//!< parameter determining the q current PID config
	PID_Controller PID_current_d;//!< parameter determining the d current PID config
	LPF_Controller LPF_current_q;//!<  parameter determining the current Low pass filter configuration 
	LPF_Controller LPF_current_d;//!<  parameter determining the current Low pass filter configuration 

	Legacy_Current_Controller();

	//Set shaft velocity in monitor pointers (index 1)
	virtual void linkMotor(FOCMotor *in_motor = nullptr) override;


	int tuneCurrentController(float bandwidth);

	//Input : current setpoint
	//Output: 0 if ok, NOT_SET if error
	//Needs: 
	// - motor->electrical_angle for DC current and FOC current
	// - shaft_velocity for estimated current and DC current
	//Will set motor D and Q voltages
	virtual float Control(float input, float dt) override;
	virtual void Command(parsed_user_command command, Commander &source) override;
};


