#include "Legacy_Current_Controller.h"
#include <common/foc_utils.h>
#include <communication/SimpleFOCDebug.h>
#include <communication/Commander.h>

Legacy_Current_Controller::Legacy_Current_Controller(/* args */)
	:PID_current_q(DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY),
	PID_current_d(DEF_PID_CURR_P, DEF_PID_CURR_I, DEF_PID_CURR_D, DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY),
	LPF_current_q(DEF_CURR_FILTER_Tf),
	LPF_current_d(DEF_CURR_FILTER_Tf)
{
}


void Legacy_Current_Controller::linkMotor(FOCMotor *in_motor)
{
	Motor_Controller::linkMotor(in_motor);
	if (motor == in_motor)
	{
		motor->monitor_pointers[1] = &shaft_velocity;
	}
}


int Legacy_Current_Controller::tuneCurrentController(float bandwidth)
{
#if SIMPLEFOC_SPLIT_CONTROLLER
	const auto &control_time = motor->control_time_us;
#else
	const auto &control_time = motor->loopfoc_time_us;
#endif
	auto old_status = motor->motor_status;
	motor->motor_status = FOCMotorStatus::motor_calibrating;

	if (bandwidth <= 0.0f) {
		// check bandwidth is positive
		SIMPLEFOC_MOTOR_ERROR("Fail. BW <= 0");
		return 1;
	}
	if (control_time && bandwidth > 0.5f * (1e6f / control_time)) {
		// check bandwidth is not too high for the control loop frequency
		SIMPLEFOC_MOTOR_ERROR("Fail. BW too high, current loop freq:" , (1e6f / control_time));
		return 2;
	}
	if (!_isset(motor->phase_resistance) || (!_isset(motor->phase_inductance) && !_isset(motor->axis_inductance.q))) {
		// need motor parameters to tune the controller
		SIMPLEFOC_MOTOR_WARN("Motor params missing!");
		if(motor->characteriseMotor( motor->voltage_sensor_align )) { 
		return 3;
		}
	}else if (_isset(motor->phase_inductance) && !(_isset(motor->axis_inductance.q))) {
		// if only single inductance value is set, use it for both d and q axis
		motor->axis_inductance = {motor->phase_inductance, motor->phase_inductance};
	}

	PID_current_q.P = motor->axis_inductance.q * (_2PI * bandwidth);
	PID_current_q.I = motor->phase_resistance * (_2PI * bandwidth);
	PID_current_d.P = _isset(motor->axis_inductance.d) ? motor->axis_inductance.d * (_2PI * bandwidth) : motor->phase_inductance * (_2PI * bandwidth);
	PID_current_d.I = motor->phase_resistance * (_2PI * bandwidth);
	LPF_current_d.Tf = 1.0f / (_2PI * bandwidth * 5.0f); // filter cutoff at 5x bandwidth
	LPF_current_q.Tf = 1.0f / (_2PI * bandwidth * 5.0f); // filter cutoff at 5x bandwidth

	SIMPLEFOC_MOTOR_DEBUG("Tunned PI params for BW [Hz]: ", bandwidth);
	SIMPLEFOC_MOTOR_DEBUG("Pq: ", PID_current_q.P);
	SIMPLEFOC_MOTOR_DEBUG("Iq: ", PID_current_q.I);
	SIMPLEFOC_MOTOR_DEBUG("Pd: ", PID_current_d.P);
	SIMPLEFOC_MOTOR_DEBUG("Id: ", PID_current_d.I);

	motor->motor_status = old_status;
	return 0;
}

float Legacy_Current_Controller::Control(float input, float dt)
{
    if (motor->sensor)
	{
		motor->sensor->update();
	}

	if (!motor->enabled)
	{
		return 0;
	}
	
	
	if (isnanf(input))
	{
		SimpleFOCDebug::println("Legacy Current Controller Error: Current setpoint or shaft velocity NAN!");
		return NOT_SET;
	}
	
	//loopFOC part -------------------------------------------------------------------------------------	

	switch (torque_controller) {
		case TorqueControlType::voltage:
			motor->voltage.q = _constrain(current_setpoint, -motor->voltage_limit, motor->voltage_limit) + feed_forward_voltage.q;
			motor->voltage.d = feed_forward_voltage.d;
		break;
		case TorqueControlType::estimated_current:
			if(! _isset(motor->phase_resistance)) {
				SimpleFOCDebug::println("Legacy Current Controller Error: Missing phase resistance!");
				return NOT_SET; 
			}
            if (isnanf(shaft_velocity))
            {
                SimpleFOCDebug::println("Legacy Current Controller Error: NAN shaft velocity!");
				return NOT_SET; 
            }
            
			// constrain current setpoint
			current_setpoint = _constrain(current_setpoint, -motor->current_limit, motor->current_limit)  + feed_forward_current.q; // desired current is the setpoint
			// calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
			if (_isset(motor->KV_rating)) motor->voltage_bemf = motor->estimateBEMF(shaft_velocity);
			// filter the value values
			motor->current.q = LPF_current_q.Control(current_setpoint, dt);
			// calculate the phase voltage
			motor->voltage.q = motor->current.q * motor->phase_resistance + motor->voltage_bemf;
			// constrain voltage within limits
			motor->voltage.q = _constrain(motor->voltage.q, -motor->voltage_limit, motor->voltage_limit) + feed_forward_voltage.q;
			// d voltage  - lag compensation 
			if(_isset(motor->axis_inductance.q)) motor->voltage.d = _constrain( -current_setpoint*shaft_velocity*motor->pole_pairs*motor->axis_inductance.q, -motor->voltage_limit, motor->voltage_limit) + feed_forward_voltage.d;
			else motor->voltage.d = feed_forward_voltage.d;
		break;
		case TorqueControlType::dc_current:
			if(!motor->current_sense) {
				SimpleFOCDebug::println("Legacy Current Controller Error: Missing current sense!");
				return NOT_SET;
			}
            if (isnanf(shaft_velocity))
            {
                SimpleFOCDebug::println("Legacy Current Controller Error: NAN shaft velocity!");
				return NOT_SET; 
            }
            if (isnanf(motor->electrical_angle))
            {
                SimpleFOCDebug::println("Legacy Current Controller Error: NAN electrical angle!");
				return NOT_SET; 
            }
			// constrain current setpoint
			current_setpoint = _constrain(current_setpoint, -motor->current_limit, motor->current_limit) + feed_forward_current.q;
			// read overall current magnitude
			motor->current.q = motor->current_sense->getDCCurrent(motor->electrical_angle);
			// filter the value values
			motor->current.q = LPF_current_q.Control(motor->current.q, dt);
			// calculate the phase voltage
			motor->voltage.q = PID_current_q.Control(current_setpoint - motor->current.q, dt) + feed_forward_voltage.q;
			// d voltage  - lag compensation
			if(_isset(motor->axis_inductance.q)) motor->voltage.d = _constrain( -current_setpoint*shaft_velocity*motor->pole_pairs*motor->axis_inductance.q, -motor->voltage_limit, motor->voltage_limit) + feed_forward_voltage.d;
			else motor->voltage.d = feed_forward_voltage.d;
		break;
		case TorqueControlType::foc_current:
			if(!motor->current_sense) {
				SimpleFOCDebug::println("Legacy Current Controller Error: Missing current sense!");
				return NOT_SET;
			}
            if (isnanf(motor->electrical_angle))
            {
                SimpleFOCDebug::println("Legacy Current Controller Error: NAN electrical angle!");
				return NOT_SET; 
            }
			// constrain current setpoint
			current_setpoint = _constrain(current_setpoint, -motor->current_limit, motor->current_limit) + feed_forward_current.q;
			// read dq currents
			motor->current = motor->current_sense->getFOCCurrents(motor->electrical_angle);
			// filter values
			motor->current.q = LPF_current_q.Control(motor->current.q, dt);
			motor->current.d = LPF_current_d.Control(motor->current.d, dt); 
			// calculate the phase voltages
			motor->voltage.q = PID_current_q.Control(current_setpoint - motor->current.q, dt);
			motor->voltage.d = PID_current_d.Control(feed_forward_current.d - motor->current.d, dt);
			// d voltage - lag compensation
			//if(_isset(axis_inductance.q)) voltage.d = _constrain( voltage.d - current_setpoint*shaft_velocity*pole_pairs*axis_inductance.q, -voltage_limit, voltage_limit);
			// q voltage - cross coupling compensation - TODO verify
			//if(_isset(axis_inductance.d)) voltage.q = _constrain( voltage.q + current.d*shaft_velocity*pole_pairs*axis_inductance.d, -voltage_limit, voltage_limit);
			// add feed forward
			motor->voltage.q += feed_forward_voltage.q;
			motor->voltage.d += feed_forward_voltage.d;
		break;
		default:
		// no torque control selected
		SimpleFOCDebug::println("Legacy Current Controller Error: No torque control selected!");
		break;
	}
	return 0;
}

void Legacy_Current_Controller::Command(parsed_user_command command, Commander &source)
{
#if SIMPLEFOC_SPLIT_CONTROLLER
	switch (command.command_chars[0])
	{
	case CMD_C_Q_PID:      //
		source.printVerbose(F("PID curr q| "));
		if(command.command_chars[1] == SCMD_LPF_TF) source.lpf(&LPF_current_q, command.consume_command_chars(1));
		else source.pid(&PID_current_q, command.consume_command_chars(1));
		break;
	case CMD_C_D_PID:      //
		source.printVerbose(F("PID curr d| "));
		if(command.command_chars[1] == SCMD_LPF_TF) source.lpf(&LPF_current_d, command.consume_command_chars(1));
		else source.pid(&PID_current_d, command.consume_command_chars(1));
		break;
	case CMD_TORQUE_TYPE:
		// change control type
		source.printVerbose(F("Torque: "));
		switch(command.command_chars[1]){
		case SCMD_TIME:
			source.printVerbose(F(" time: "));
			#if SIMPLEFOC_SPLIT_CONTROLLER
			source.println((int)motor->control_time_us);
			#else
			source.println((int)motor->loopfoc_time_us);
			#endif
			break;
		default:
			if(command.has_data && (int8_t)command.value >= 0 && (int8_t)command.value < 4) // if set command
				torque_controller = static_cast<TorqueControlType>(command.value); // update torque control type
			switch(torque_controller){
			case TorqueControlType::voltage:
				source.println(F("volt"));
				break;
			case TorqueControlType::dc_current:
				source.println(F("dc curr"));
				break;
			case TorqueControlType::foc_current:
				source.println(F("foc curr"));
				break;
			case TorqueControlType::estimated_current:
				source.println(F("est. curr"));
				break;
			default:
				source.printError();
				break;
			}
			break;
		}
		break;
	case CMD_STATUS:
		// enable/disable
		source.printVerbose(F("Status: "));
		if(command.has_data) (bool)command.value ? motor->enable() : motor->disable();
		source.println(motor->enabled);
		break;
	case CMD_FOC_PARAMS:
		switch (command.command_chars[1])
		{
		case SCMD_TUNE_CURR:
			tuneCurrentController(command.value);
			break;
		
		default:
			break;
		}
	default:
		break;
	}
#endif
}
