/* GenericStepper.cpp */
#include "GenericStepper.h"
#include <math.h>

StepMode::StepMode()
{
}

StepMode::StepMode(int step_mode[])
{
	mode_0 = step_mode[0];
	mode_1 = step_mode[1];
	mode_2 = step_mode[2];
}

StepModeManager::StepModeManager()
{
}

StepModeManager::StepModeManager(int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin)
{
	_current_step_mode_index = 0;
	_step_mode_0_pin = step_mode_0_pin;
	_step_mode_1_pin = step_mode_1_pin;
	_step_mode_2_pin = step_mode_2_pin;
}

int StepModeManager::_set_step_mode(int step_mode_index, int full_step_count)
{
	//std::cout << "_set_step_mode()\n_step_mode_count : " << _step_mode_count << "\n";
	int new_step_count;
	step_mode_index = constrain(step_mode_index, 0, _step_mode_count - 1);
	//std::cout<< "step_mode_index : " << step_mode_index << "\n";
	digitalWrite(_step_mode_0_pin, _step_modes[step_mode_index][0]);
	digitalWrite(_step_mode_1_pin, _step_modes[step_mode_index][1]);
	digitalWrite(_step_mode_2_pin, _step_modes[step_mode_index][2]);
	new_step_count = full_step_count * pow(2, step_mode_index);
	//std::cout << "new step mode : " << _step_modes[step_mode_index][0] << " | " << _step_modes[step_mode_index][1] << " | " << _step_modes[step_mode_index][2] << "\n";
	//std::cout << "new_step_count : " << new_step_count << "\n";
	return new_step_count;

}

DRV8825StepModeManager::DRV8825StepModeManager(int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin) : StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin)
{
	_step_mode_count = DRV8825_step_mode_count;
	for (int step_mode_index = 0; step_mode_index < 8; step_mode_index++)
	{
		for (int step_mode_pin_index = 0; step_mode_pin_index < 3; step_mode_pin_index++)
		{
			_step_modes[step_mode_index][step_mode_pin_index] = DRV8825_step_modes[step_mode_index][step_mode_pin_index];
		}
	}
}

A4988StepModeManager::A4988StepModeManager()
{
}

A4988StepModeManager::A4988StepModeManager(int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin) : StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin)
{
	_step_mode_count = A4988_step_mode_count;
	for (int step_mode_index = 0; step_mode_index < 8; step_mode_index++)
	{
		for (int step_mode_pin_index = 0; step_mode_pin_index < 3; step_mode_pin_index++)
		{
			_step_modes[step_mode_index][step_mode_pin_index] = A4988_step_modes[step_mode_index][step_mode_pin_index];
		}
	}
}
/*
GenericStepper::GenericStepper()
{
}
*/
GenericStepper::GenericStepper(int full_step_count, int step_pin, int dir_pin, int not_sleep_pin, int not_reset_pin, int not_enable_pin, int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin)
{
	_full_step_count = full_step_count;
	_step_count = full_step_count;

	_step_pin = step_pin;
	_dir_pin = dir_pin;
	_not_sleep_pin = not_sleep_pin;
	_not_reset_pin = not_reset_pin;
	_not_enable_pin = not_enable_pin;

	_step_mode_manager = StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin);

	_direction = LOW;
	_not_sleeping = true;
	_not_enabled = true;

	_min_step_time = 2;
	_max_step_time = round(1000000 / _step_count); // _max_step_time is the step time required for the motor to run at 1 rpm
	_step_time = _max_step_time;

	_max_num_variation_steps = num_step_time_variation_coefficients;
	_max_speed = _step_count / _microsec_to_sec(_min_step_time);
}

int GenericStepper::get_speed()
{
	return (_speed * MINUTE) / _step_count;
}

void GenericStepper::set_speed(int new_rpm_speed)
{
	_speed = (new_rpm_speed * _step_count) / MINUTE;
	_step_time = 1000000 / _speed;
	//std::cout << "_speed : " << _speed << "\n_step_time : " << _step_time << "\n";
}

int GenericStepper::get_pos()
{
	return _position;
}

void GenericStepper::set_pos(int new_position)
{
	int num_steps;
	int num_revolutions = 1;
	if (new_position > _step_count)
	{
		num_revolutions = (new_position - (new_position % _step_count)) / _step_count;
	}
	if (_direction == HIGH)
	{
		if (new_position > _position)
		{
			num_steps = new_position - _position;
		}
		else
		{
			num_steps = _step_count - (_position - new_position);
		}
	}
	else
	{
		if (new_position < _position)
		{
			num_steps = _position - new_position;
		}
		else
		{
			num_steps = _step_count - (new_position - _position);
		}
	}

	bool accel = true;
	bool decel = true;
	if (num_revolutions > 0)
	{
		accel = false;
		if (num_steps > 0)
		{
			decel = false;
		}
		else
		{
			decel = true;
		}
		_run_revolution_steps(num_revolutions, true, decel);
	}
	else
	{
		accel = true;
	}
	run_steps(num_steps, accel, decel);
}

void GenericStepper::run_steps(int num_steps, bool steps_accel, bool steps_decel)
{
	int num_accel_steps = 0;
	int num_decel_steps = 0;
	if (steps_accel == true)
	{
		num_accel_steps = _get_num_speed_variation_steps(num_steps, acceleration_ratio);
	}
	if (steps_decel == true)
	{
		num_decel_steps = _get_num_speed_variation_steps(num_steps, deceleration_ratio);
	}
	int num_cruise_steps = num_steps - (num_accel_steps + num_decel_steps);
	//std::cout << "accel_steps : " << num_accel_steps << "\n" << "decel_steps : " << num_decel_steps << "\n" << "cruise_steps : " << num_cruise_steps << "\n";
	if (steps_accel == true)
	{
		_accelerate(num_accel_steps);
	}
	for (int step_time_index = 0; step_time_index < num_cruise_steps; step_time_index++)
	{
		_run_step(_step_time);
		//std::cout << "full speed step :  " << _step_time << " microseconds\n";
	}
	if (steps_decel == true)
	{
		_decelerate(num_accel_steps);
	}
};

void GenericStepper::set_step_mode(int step_mode_index)
{
	_step_count = _step_mode_manager._set_step_mode(step_mode_index, _full_step_count);
	_max_step_time = round(1000000 / _step_count);
}

int GenericStepper::get_dir()
{
	return _direction;
}

void GenericStepper::set_dir(int new_direction)
{
	_direction = new_direction;
	digitalWrite(_dir_pin, new_direction);
	delayMicroseconds(2);
}

void GenericStepper::enable()
{
	_not_enabled = false;
	digitalWrite(_not_enable_pin, LOW);
}

void GenericStepper::disable()
{
	_not_enabled = true;
	digitalWrite(_not_enable_pin, HIGH);
}

void GenericStepper::wake()
{
	_not_sleeping = true;
	digitalWrite(_not_sleep_pin, HIGH);
}

void GenericStepper::sleep()
{
	_not_sleeping = false;
	digitalWrite(_not_sleep_pin, LOW);
}

void GenericStepper::_run_revolution_steps(int num_revolutions, bool revs_accel, bool revs_decel)
{
	bool accel = true;
	bool decel = true;
	for (int rev_index = 0; rev_index < num_revolutions; rev_index++)
	{
		if (rev_index == 0)
		{
			accel = revs_accel;
		}
		if (rev_index == num_revolutions - 1)
		{
			decel = revs_decel;
		}
		run_steps(_step_count, accel, decel);
	}
}

void GenericStepper::_init_pins()
{
	pinMode(_step_pin, OUTPUT);
	pinMode(_dir_pin, OUTPUT);
	pinMode(_not_sleep_pin, OUTPUT);
	pinMode(_not_reset_pin, OUTPUT);
	pinMode(_not_reset_pin, OUTPUT);

	digitalWrite(_step_pin, LOW);
	digitalWrite(_dir_pin, _direction);
	wake();
	disable();
	digitalWrite(_not_reset_pin, HIGH);
	set_step_mode(0);
}

void GenericStepper::_run_step(int step_time)
{
//	//std::cout << "step_time : " << step_time << '\n';
	digitalWrite(_step_pin, HIGH);
	delayMicroseconds(1);
	digitalWrite(_step_pin, LOW);
	delayMicroseconds(step_time);
	if (_direction == true)
	{
		_position += 1;
	}
	else
	{
		_position -= 1;
	}
}

int GenericStepper::_microsec_to_sec(int time_in_microsecs)
{
	return time_in_microsecs * 1000000;
}

int GenericStepper::_get_num_speed_variation_steps(int num_steps, float speed_variation_ratio)
{
	int num_speed_variation_steps = (num_steps - (num_steps % (int(round(100/speed_variation_ratio)/100)))) / (int(round(100/speed_variation_ratio)/100));
	if (num_speed_variation_steps > _max_num_variation_steps)
	{
		num_speed_variation_steps = _max_num_variation_steps;
	}
	return num_speed_variation_steps;
}

void GenericStepper::_accelerate(int num_steps)
{
	int step_time_variation;
	int current_step_time;
	int step_time_jump;
	int fix_step_time_jump_from;
	int fix_step_time_jump_until;
	if (num_steps == _max_num_variation_steps)
	{
		for (int step_time_index = 0; step_time_index < num_steps; step_time_index++)
		{
			step_time_variation = _max_step_time * step_time_variation_coefficients[step_time_index];
			current_step_time = _min_step_time + round(step_time_variation);
			//std::cout << "current step time : " << current_step_time << "\n" << "step time variation : " << step_time_variation << "\n";
		}
	}
	else
	{
		int step_time_jump_data[3] = {0, 0, 0};
		int current_step_time_jump;
		_set_speed_variation_step_time_jump(num_steps, step_time_jump_data);
		step_time_jump = step_time_jump_data[0];
		fix_step_time_jump_from = step_time_jump_data[1];
		fix_step_time_jump_until = step_time_jump_data[2];
		current_step_time_jump = step_time_jump;
		for (int step_time_index = 0; step_time_index < _max_num_variation_steps; step_time_index = step_time_index + current_step_time_jump)
		{

			step_time_variation = _max_step_time * step_time_variation_coefficients[step_time_index];
			current_step_time = _step_time + round(step_time_variation);
			//std::cout << step_time_index << " : current step time : " << current_step_time;// << "\n\tstep time variation : " << step_time_variation << "\n\tstep_time_jump : " << current_step_time_jump << "\n";

			_run_step(current_step_time);
			if ((step_time_index > fix_step_time_jump_from) & (step_time_index < fix_step_time_jump_until))
			{
				current_step_time_jump = step_time_jump + 1;
				//std::cout << "\t | inside fix_step_time_jump window\n";
			}
			else
			{
				current_step_time_jump = step_time_jump;
				//std::cout << "\n";
			}
		}
	}
}

void GenericStepper::_decelerate(int num_steps)
{
	int step_time_variation;
	int current_step_time;
	int step_time_jump;
	int fix_step_time_jump_from;
	int fix_step_time_jump_until;
	if (num_steps == _max_num_variation_steps)
	{
		for (int step_time_index = num_steps; step_time_index > 0; step_time_index--)
		{
			current_step_time = _min_step_time + round(_max_step_time * step_time_variation_coefficients[step_time_index]);
		}
	}
	else
	{
		int step_time_jump_data[3] = {0, 0, 0};
		int current_step_time_jump;
		_set_speed_variation_step_time_jump(num_steps, step_time_jump_data);
		step_time_jump = step_time_jump_data[0];
		fix_step_time_jump_from = step_time_jump_data[1];
		fix_step_time_jump_until = step_time_jump_data[2];
		current_step_time_jump = step_time_jump;
		for (int step_time_index = _max_num_variation_steps - 1; step_time_index >= 0; step_time_index = step_time_index - current_step_time_jump)
		{
			step_time_variation = _max_step_time * step_time_variation_coefficients[step_time_index];
			current_step_time = _step_time + round(step_time_variation);
			//std::cout << step_time_index << " : current step time : " << current_step_time;// << "\n\tstep time variation : " << step_time_variation << "\n\tstep_time_jump : " << current_step_time_jump << "\n";
			_run_step(current_step_time);
			if ((step_time_index > fix_step_time_jump_from) & (step_time_index < fix_step_time_jump_until))
			{
				current_step_time_jump = step_time_jump + 1;
				//std::cout << "\t | inside fix_step_time_jump window\n";
			}
			else
			{
				current_step_time_jump = step_time_jump;
				//std::cout << "\n";
			}
		}
	}
}

void GenericStepper::_set_speed_variation_step_time_jump(int num_steps, int * step_time_jump_data)
{
	int num_remaining_step_times;
	int step_time_jump;
	int fix_step_time_jump_from;
	int fix_step_time_jump_until;

	//std::cout << "setting speed variation step time jump\n";
	num_remaining_step_times = _max_num_variation_steps % num_steps;
	step_time_jump = (_max_num_variation_steps - ( _max_num_variation_steps % num_steps)) / num_steps;
	fix_step_time_jump_from = (_max_num_variation_steps / 2) - (num_remaining_step_times / 2);
	fix_step_time_jump_until = (_max_num_variation_steps / 2) + (num_remaining_step_times / 2);
	//std::cout << "setting step_time_jump_data\n";
	step_time_jump_data[0] = step_time_jump;
	step_time_jump_data[1] = fix_step_time_jump_from;
	step_time_jump_data[2] = fix_step_time_jump_until;
	//std::cout << "step_time_jump_data set\n";
}

DRV8825Stepper::DRV8825Stepper(int step_count, int step_pin, int dir_pin, int not_sleep_pin, int not_reset_pin, int not_enable_pin, int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin, int no_fault_pin) : GenericStepper(step_count, step_pin, dir_pin, not_sleep_pin, not_reset_pin, not_enable_pin, step_mode_0_pin, step_mode_1_pin, step_mode_2_pin)
{
	_no_fault_pin = no_fault_pin;
	_step_mode_manager = DRV8825StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin);
	_init_pins();
};

A4988Stepper::A4988Stepper(int step_count, int step_pin, int dir_pin, int not_sleep_pin, int not_reset_pin, int not_enable_pin, int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin) : GenericStepper(step_count, step_pin, dir_pin, not_sleep_pin, not_reset_pin, not_enable_pin, step_mode_0_pin, step_mode_1_pin, step_mode_2_pin)
{
	_step_mode_manager = A4988StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin);
	_init_pins();
};

void DRV8825Stepper::_init_pins()
{
	pinMode(_step_pin, OUTPUT);
	pinMode(_dir_pin, OUTPUT);
	pinMode(_not_sleep_pin, OUTPUT);
	pinMode(_not_reset_pin, OUTPUT);
	pinMode(_not_reset_pin, OUTPUT);
	pinMode(_no_fault_pin, INPUT);

	digitalWrite(_step_pin, LOW);
	digitalWrite(_dir_pin, _direction);
	wake();
	disable();
	digitalWrite(_not_reset_pin, HIGH);
	set_step_mode(0);
}

void DRV8825Stepper::_run_step(int step_time)
{
	if (digitalRead(_no_fault_pin) == HIGH)
	{
		digitalWrite(_step_pin, HIGH);
		delayMicroseconds(1);
		digitalWrite(_step_pin, LOW);
		delayMicroseconds(step_time);
	}
	if (_direction == true)
	{
		_position += 1;
	}
	else
	{
		_position -= 1;
	}
}
