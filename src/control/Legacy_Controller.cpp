#include "Legacy_Controller.h"
#include <common/foc_utils.h>
#include <communication/SimpleFOCDebug.h>
#include <communication/Commander.h>

Legacy_Controller::Legacy_Controller()
	: 
	PID_velocity(DEF_PID_VEL_P, DEF_PID_VEL_I, DEF_PID_VEL_D, DEF_PID_VEL_RAMP, DEF_PID_VEL_LIMIT),
	P_angle(DEF_P_ANGLE_P, 0, 0, 0, DEF_VEL_LIM),
	LPF_velocity(DEF_VEL_FILTER_Tf),
	LPF_angle(0.0f)
{

}

void Legacy_Controller::linkMotor(FOCMotor *in_motor)
{
	Legacy_Current_Controller::linkMotor(in_motor);
	if (motor == in_motor)
	{
		motor->monitor_pointers[6] = &target;
	}
}

float Legacy_Controller::Control(float input, float dt)
{
	if (motor == nullptr) [[unlikely]]
	{
		SimpleFOCDebug::println("Legacy Controller Error: No motor given!");
		return NOT_SET;
	}

	// Move part ------------------------------------------

	// set internal target variable
	if(_isset(input) ) target = input;

	float shaft_angle_sp = NAN, shaft_velocity_sp = NAN;
	
	// downsampling (optional)
	if(motion_cnt++ < motion_downsample) goto loopfoc;
	motion_cnt = 0;

	// if initFOC is being executed at the moment, do nothing
	if(motor->motor_status == FOCMotorStatus::motor_calibrating) return NOT_SET;
	
	// calculate the elapsed time between the calls
	// TODO replace downsample by runnind the code at 
	// a specific frequency (or almost)

	// read value even if motor is disabled to keep the monitoring updated
	// except for the open loop modes where the values are updated within angle/velocityOpenLoop functions
	
	// shaft angle/velocity need the update() to be called first
	// get shaft angle
	// TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
	//                        For this reason it is NOT precise when the angles become large.
	//                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
	//                        when switching to a 2-component representation.
	if( controller != MotionControlType::angle_openloop && controller != MotionControlType::velocity_openloop ){
		// read the values only if the motor is not in open loop
		// because in open loop the shaft angle/velocity is updated within angle/velocityOpenLoop functions
		motor->shaft_angle = motor->shaftAngle();
		shaft_velocity = LPF_velocity.Control(motor->sensor->getVelocity(), dt); 
	}

	// if disabled do nothing
	// and if 
	if(!motor->enabled) goto loopfoc;

	current_setpoint = NAN;
	

	// upgrade the current based voltage limit
	switch (controller) {
		case MotionControlType::torque:
			current_setpoint = target;
			break;
		case MotionControlType::angle_nocascade:
			// TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
			//                        the angles are large. This results in not being able to command small changes at high position values.
			//                        to solve this, the delta-angle has to be calculated in a numerically precise way.
			// angle set point
			shaft_angle_sp = target;
			// calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
			current_setpoint = P_angle.Control(shaft_angle_sp - LPF_angle.Control(motor->shaft_angle, dt), dt);
			break;
		case MotionControlType::angle:
			// TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
			//                        the angles are large. This results in not being able to command small changes at high position values.
			//                        to solve this, the delta-angle has to be calculated in a numerically precise way.
			// angle set point
			shaft_angle_sp = target;
			// calculate velocity set point
			shaft_velocity_sp = feed_forward_velocity + P_angle.Control(shaft_angle_sp - LPF_angle(motor->shaft_angle, dt), dt);
			shaft_velocity_sp = _constrain(shaft_velocity_sp, -motor->velocity_limit, motor->velocity_limit);
			// calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
			current_setpoint = PID_velocity.Control(shaft_velocity_sp - shaft_velocity, dt); 
			break;
		case MotionControlType::velocity:
			// velocity set point - sensor precision: this calculation is numerically precise.
			shaft_velocity_sp = target;
			// calculate the torque command
			current_setpoint = PID_velocity.Control(shaft_velocity_sp - shaft_velocity, dt); 
			break;
		case MotionControlType::velocity_openloop:
			// velocity control in open loop - sensor precision: this calculation is numerically precise.
			shaft_velocity_sp = target;
			// this function updates the shaft_angle and shaft_velocity
			// returns the voltage or current that is to be set to the motor (depending on torque control mode)
			// returned values correspond to the voltage_limit and current_limit
			// calculate the necessary angle to achieve target velocity
			motor->shaft_angle = _normalizeAngle(motor->shaft_angle + shaft_velocity_sp*dt);
			// for display purposes
			shaft_velocity = shaft_velocity_sp;

			if (torque_controller == TorqueControlType::voltage)
				current_setpoint = motor->voltage_limit;
			else
				current_setpoint = motor->current_limit;
			break;
		case MotionControlType::angle_openloop:
			// angle control in open loop - 
			// TODO sensor precision: this calculation NOT numerically precise, and subject
			//                        to the same problems in small set-point changes at high angles 
			//                        as the closed loop version.
			shaft_angle_sp = target;
			// this function updates the shaft_angle and shaft_velocity
			// returns the voltage or current that is to be set to the motor (depending on torque control mode)
			// returned values correspond to the voltage_limit and current_limit
			current_setpoint = motor->angleOpenloop(shaft_angle_sp, dt); 
			break;
		default:
			SimpleFOCDebug::println("Legacy Controller Error: Unknown motion mode!");
			break;
	}

loopfoc:

	if (motor->enabled)
	{
		// if open-loop do nothing
		if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) 
			// calculate the open loop electirical angle
			motor->electrical_angle = _electricalAngle(motor->shaft_angle, motor->pole_pairs);
		else
			// Needs the update() to be called first
			// This function will not have numerical issues because it uses Sensor::getMechanicalAngle() 
			// which is in range 0-2PI
			motor->electrical_angle = motor->electricalAngle();
	}

	return Legacy_Current_Controller::Control(current_setpoint, dt);
}

void Legacy_Controller::Command(parsed_user_command command, Commander &source)
{
#if SIMPLEFOC_SPLIT_CONTROLLER
	if (command.num_command_chars == 0)
	{
		if (command.has_data)
		{
			target = command.value;
			source.printVerbose(F("Target: "));
  			source.println(target);
		}
		else
		{
			source.printlnMachineReadable(target);
		}
		return;
	}
	
	switch (command.command_chars[0])
	{
	case CMD_V_PID:      //
		source.printVerbose(F("PID vel| "));
		if(command.command_chars[1] == SCMD_LPF_TF) source.lpf(&LPF_velocity, command.consume_command_chars(1));
		else source.pid(&PID_velocity, command.consume_command_chars(1));
		break;
	case CMD_A_PID:      //
		source.printVerbose(F("PID angle| "));
		if(command.command_chars[1] == SCMD_LPF_TF) source.lpf(&LPF_angle, command.consume_command_chars(1));
		else source.pid(&P_angle, command.consume_command_chars(1));
		break;
	case CMD_MOTION_TYPE:
		source.printVerbose(F("Motion:"));
		switch(command.command_chars[1]){
		case SCMD_TIME:
			source.printVerbose(F(" time: "));
			#if SIMPLEFOC_SPLIT_CONTROLLER
			source.println((int)motor->control_time_us);
			#else
			source.println((int)motor->loopfoc_time_us);
			#endif
			break;
		case SCMD_DOWNSAMPLE:
			source.printVerbose(F(" downsample: "));
			if(command.has_data) motion_downsample = command.value;
			source.println((int)motion_downsample);
		break;
		default:
			// change control type
			if(command.has_data && command.value >= 0 && (int)command.value < 7) // if set command
				controller = static_cast<MotionControlType>(command.value); // update motion control type
			switch(controller){
				case MotionControlType::torque:
				source.println(F("torque"));
				break;
				case MotionControlType::velocity:
				source.println(F("vel"));
				break;
				case MotionControlType::angle:
				source.println(F("angle"));
				break;
				case MotionControlType::velocity_openloop:
				source.println(F("vel open"));
				break;
				case MotionControlType::angle_openloop:
				source.println(F("angle open"));
				break;
				case MotionControlType::angle_nocascade:
				source.println(F("angle nocascade"));
				break;
				case MotionControlType::custom:
				source.println(F("custom"));
				break;
			}
			break;
		}
	  break;
	default:
		Legacy_Current_Controller::Command(command, source);
		break;
	}
#endif
}