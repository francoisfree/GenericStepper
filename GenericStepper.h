/* GenericStepper.h */

#ifndef GenericStepper_h
#define GenericStepper_h

#define MINUTE 60

#include "Arduino.h"
#include <math.h>

#define HIGH 0x1
#define LOW 0x0

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

const double acceleration_steps[] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.02, 0.02, 0.03, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1, 0.12, 0.14, 0.17, 0.2, 0.23, 0.27, 0.31, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.69, 0.73, 0.77, 0.8, 0.83, 0.86, 0.88, 0.9, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.97, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99};

//const double step_time_variation_coefficients[] = {0.9918374, 0.9900482, 0.9878716, 0.985226, 0.9820138, 0.9781187, 0.973403, 0.9677045, 0.9608343, 0.9525741, 0.9426758, 0.9308616, 0.9168273, 0.9002495, 0.8807971, 0.8581489, 0.8320184, 0.8021839, 0.7685248, 0.7310586, 0.6899745, 0.6456563, 0.5986877, 0.549834, 0.5, 0.450166, 0.4013123, 0.3543437, 0.3100255, 0.2689414, 0.2314752, 0.1978161, 0.1679816, 0.1418511, 0.1192029, 0.0997505, 0.0831727, 0.0691384, 0.0573242, 0.0474259, 0.0391657, 0.0322955, 0.026597, 0.0218813, 0.0179862, 0.014774, 0.0121284, 0.0099518, 0.0081626, 0.0066929};

const double step_time_variation_coefficients[] = {0.9999032, 0.9998909, 0.9998759, 0.9998576, 0.9998352, 0.9998079000000001, 0.9997745, 0.9997338, 0.999684, 0.9996232, 0.999549, 0.9994584000000001, 0.9993477000000001, 0.9992126, 0.9990475, 0.9988461000000001, 0.9986001, 0.9982998000000001, 0.9979333, 0.997486, 0.9969402, 0.9962744, 0.9954623, 0.9944723, 0.9932657, 0.991796, 0.9900068000000001, 0.9878302, 0.9851846000000001, 0.9819724000000001, 0.9780773, 0.9733616, 0.9676631, 0.9607929000000001, 0.9525327, 0.9426344, 0.9308202, 0.9167859, 0.9002081000000001, 0.8807557, 0.8581075, 0.8319770000000001, 0.8021425, 0.7684834, 0.7310172, 0.6899331000000001, 0.6456149, 0.5986463000000001, 0.5497926000000001, 0.4999586, 0.4501246, 0.4012709, 0.35430229999999996, 0.3099841, 0.2689, 0.2314338, 0.1977747, 0.1679402, 0.1418097, 0.1191615, 0.09970910000000001, 0.0831313, 0.069097, 0.0572828, 0.0473845, 0.0391243, 0.0322541, 0.0265556, 0.0218399, 0.0179448, 0.0147326, 0.012086999999999999, 0.0099104, 0.0081212, 0.0066515, 0.005444900000000001, 0.0044549, 0.0036428, 0.002977, 0.0024312, 0.0019839, 0.0016174, 0.0013170999999999999, 0.0010711, 0.0008696999999999999, 0.0007046, 0.0005695, 0.00045880000000000004, 0.0003682, 0.00029400000000000004, 0.0002332, 0.00018339999999999999, 0.0001427, 0.0001093, 8.199999999999999e-05, 5.9600000000000005e-05, 4.130000000000001e-05, 2.630000000000001e-05, 1.4000000000000001e-05, 4.000000000000002e-06};

const int num_step_time_variation_coefficients = 100;
const float acceleration_ratio = 0.2;
const float deceleration_ratio = 0.2;

const int A4988_step_mode_count = 5;
const int DRV8825_step_mode_count = 6;

const int A4988_step_modes[8][3] = {
				{LOW, LOW, LOW},
				{HIGH, LOW, LOW},
				{LOW, HIGH, LOW},
				{HIGH, HIGH, LOW},
				{HIGH, HIGH, HIGH},
				{HIGH, HIGH, HIGH},
				{HIGH, HIGH, HIGH},
				{HIGH, HIGH, HIGH}
};

const int DRV8825_step_modes[8][3] = {
				{LOW, LOW, LOW},
				{HIGH, LOW, LOW},
				{LOW, HIGH, LOW},
				{HIGH, HIGH, LOW},
				{LOW, LOW, HIGH},
				{HIGH, LOW, HIGH},
				{LOW, HIGH, HIGH},
				{HIGH, HIGH, HIGH}
};

class StepMode
{
	public:
		StepMode();
		StepMode(int step_mode[]);
		int mode_0;
		int mode_1;
		int mode_2;
};

class StepModeManager
{
	friend class GenericStepper;
	public:
		StepModeManager();
		StepModeManager(int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin);
		// set_step_mode returns the step count for the newly set step mode
	protected:
		int _step_mode_0_pin;
		int _step_mode_1_pin;
		int _step_mode_2_pin;
		int _step_mode_count;
		int _step_modes[8][3];
		int _current_step_mode_index;
		int _set_step_mode(int step_mode_index, int full_step_count);

};

class A4988StepModeManager: public StepModeManager
{
	friend class A4988Stepper;
	public:
		A4988StepModeManager();
		A4988StepModeManager(int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin);// : StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin);
};

class DRV8825StepModeManager: public StepModeManager
{
	friend class DRV8825Stepper;
	public:
		DRV8825StepModeManager(int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin);// : StepModeManager(step_mode_0_pin, step_mode_1_pin, step_mode_2_pin);
};

class GenericStepper
{
	public:
		// Constructors
//		GenericStepper();
		GenericStepper(int full_step_count, int step_pin, int dir_pin, int not_sleep_pin, int not_reset_pin, int not_enable_pin, int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin);

		int get_speed();
		void set_speed(int new_speed);
		int get_pos();
		void set_pos(int new_position);
		void run_steps(int num_steps, bool steps_accel, bool steps_decel);
		void set_step_mode(int step_mode);
		int get_dir();
		void set_dir(int new_dir);
		void enable();
		void disable();
		void sleep();
		void wake();
		void reset();

	protected:

		bool _not_sleeping;
		bool _not_enabled;
		int _step_count;
		int _full_step_count;
		int _step_mode_count;

		int _position;
		int _direction;
		int _step_time;
		int _min_step_time;
		int _max_step_time;
		int _speed;
		int _max_speed;
		int _max_num_variation_steps;

		StepModeManager _step_mode_manager;

		int _step_pin;
		int _dir_pin;
		int _not_sleep_pin;
		int _not_reset_pin;
		int _not_enable_pin;
		//int _step_mode_0_pin;
		//int _step_mode_1_pin;
		//int _step_mode_2_pin;

		void _init_pins();
		void _run_step(int step_time);

		int _microsec_to_sec(int time_in_microsecs);
		int _get_num_speed_variation_steps(int num_steps, float speed_variation_ratio);
		void _accelerate(int num_steps);
		void _decelerate(int num_steps);
		void _set_speed_variation_step_time_jump(int num_steps, int * step_time_jump_data);
		void _run_revolution_steps(int num_revolutions, bool revs_accel, bool revs_decel);
};

class A4988Stepper: public GenericStepper
{
	public:
		A4988Stepper(int step_count, int step_pin, int dir_pin, int not_sleep_pin, int not_reset_pin, int not_enable_pin, int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin);
	private:
		A4988StepModeManager _step_mode_manager;
};

class DRV8825Stepper: public GenericStepper
{
	public:
		DRV8825Stepper(int step_count, int step_pin, int dir_pin, int not_sleep_pin, int not_reset_pin, int not_enable_pin, int step_mode_0_pin, int step_mode_1_pin, int step_mode_2_pin, int no_fault_pin);// : GenericStepper(step_count, step_pin, dir_pin, not_sleep_pin, not_reset_pin, not_enable_pin, step_mode_0_pin, step_mode_1_pin, step_mode_2_pin);
	private:
		int _no_fault_pin;
		void _init_pins();
		void _run_step(int step_time);
};
#endif




