#include "main.h"
#include "classesTesting.h"
// #include "autonFunctions.h"
#include "okapi/api.hpp"

// define classes from classesTesting.h
CustomMath customMath;
okapi::Rate loopRate;

// define controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// define miscellaneous motors, pneumatics, and tracking wheels
pros::Motor flywheel_motor(15, false);		// Flywheel motor
pros::Motor left_intake(19, true);			// Left intake motor
pros::Motor right_intake(14, false);		// Right intake motor
pros::ADIDigitalOut wings(1, LOW);			// Pneumatics to extend the pusher wings
pros::Rotation tracking_wheel_horizontal(13, false);
pros::Rotation tracking_wheel_vertical(12, false);
pros::Imu inertial_sensor(6);
pros::Rotation flywheel_sensor(7, true);

// define drivetrain motors
pros::Motor left_top_drive(2, true);
pros::Motor left_back_drive(3, false);
pros::Motor left_front_drive(4, true);
pros::Motor right_top_drive(10, false);
pros::Motor right_back_drive(9, true);
pros::Motor right_front_drive(8, false);

// define drivetrain motor groups
pros::Motor_Group left_drivetrain({left_top_drive, left_back_drive, left_front_drive});		// the three motors for the left side of the drivetrain
pros::Motor_Group right_drivetrain({right_top_drive, right_back_drive, right_front_drive}); // the three motors for the right side of the drivetrain
pros::Motor_Group intake({left_intake, right_intake});										// both intake motors

// declare functions so taht we can define them at the bottom of this page
void ForwardPID(float target, float settle_time_msec = 500, float kI_start_at_error_value = 7, int timeout_msec = -1);
void TurnPID(float target, float settle_time_msec = 500, float kI_start_at_error_value = 45, int timeout_msec = -1);
void flywheel_bang_bang();
void odometry();
void goTo();

// declare global variables
const double forward_kP = 500;
const double forward_kI = 0;
const double forward_kD = 0;

const double turn_kP = 500;
const double turn_kI = .1;
const double turn_kD = 10;

bool toggle_flywheel = 0;


double robot_angle; //"θ" in odom paper in centidegrees ex.: 180.00
double robot_position[2] = {0,0}; //[ x , y ] in inches

double Ty = 0.25; //initialize //"Tr" in odom paper, offset from vertical tracking wheel to tracking center in inches
double Tx = -2.5; //"Ts" in odom paper, offset from horizontal tracking wheel to tracking center in inches. It's negative because left is -x direction 
double y_arc; //"ΔR" in odom paper in inches
	// SPACE HAS TO BE HERE  or the x_arc says ""ΔR" in odom paper" when you hover over it for some reason
double x_arc; //"ΔS" in odom paper in inches


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// int calibrate_timer = 0;

	// while(inertial_sensor.is_calibrating())
	// {
	// 	pros::lcd::print(2, "inertial taken %d seconds to calibrate", calibrate_timer/1000);
	// 	calibrate_timer += 20;
	// 	pros::delay(20);

	// }

	// autonomous();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}

/*
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
/* Jeremy Here, ^ this ^  might be able to be avoided if we save variables to
 * an external microSD card constantly. maybe PID constants too for easy tuning?
 */

void autonomous()
{
	// PRE-MATCH AUTON:
	inertial_sensor.set_rotation(-90); // Sets 0 degrees to be infront of drive box
	intake = 95;
	ForwardPID(6); // intake ball under the bar
	intake = 0;
	ForwardPID(-36); // go backwards
	TurnPID(-135);
	ForwardPID(-48); // push team ball behind to the loading ground bar
	TurnPID(-180);
	ForwardPID(-24); // push team ball into goal

	// SKILLS RUN:
	//  for(int i = 0; i < 9; i++) //runs the fire routine 42 times (two extra for now)
	//  {
	//  	cata_motor.move_relative(150 * 3, 99); //180 Degrees = 1 fire; 150 ticks (red motor encoder units) = 180 degrees; gear_ratio = 36:12 = 3:1

	// }

	// move to other zone

	// ForwardPID(.5 * 12); //Move away from goal before sensor reset so no hit corner
	// TurnPID(-45); //Turn to easy angle before sensor reset

	// inertial_sensor.reset(); //because we start at a weird diagonal angle, this changes our absolute "forward" angle to straight horizontally toward the other zone

	// ForwardPID(8 * 12); //moves forward 8 feet into other zone
	// TurnPID(90); //turn toward goal
	// ForwardPID(2 * 12); //move toward goal
	// TurnPID(180); //turn toward bar
	// ForwardPID(2 * 12); // move toward bar
	// TurnPID(90); //turn to be parallel with the goal so we can set up our movement to shove triballs under
	// ForwardPID(2 * 12); //move parallel with the goal
	// TurnPID(0); //turn toward goal

	// ForwardPID(3 * 12); //score some triballs, baby!
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{

	// start tasks
	pros::Task odometry_task(odometry);
	pros::Task flywheel_task(flywheel_bang_bang);
	// pros::Task print_test(print_task_test);

	// decalre variables

	float drive_forward;
	float drive_turn;

	float left_drive_speed;
	float right_drive_speed;

	float const drive_turn_constant = 1.4;

	// initialize variables

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////Control Loop///////////////////////////////////;////////////////////////////;//////////////////;////////////////////////;////////////////////////////////////////////////////////////////////////////////////////////////////////////

	while (true)
	{

		// printing on the brain screen
		 pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// intake controller
		if (controller.get_digital(DIGITAL_R1)) // forward
		{
			intake = 95;
		}
		else if (controller.get_digital(DIGITAL_R2)) // reverse
		{
			intake = -95;
		}
		else
		{
			intake.brake();
		}

		// wings controller
		if (controller.get_digital(DIGITAL_A))
		{
			wings.set_value(HIGH);
		}
		else
		{
			wings.set_value(LOW);
		}

		// turns flywheel on and off
		if (controller.get_digital_new_press(DIGITAL_L1))
		{ // function to toggle flywheel when A is pressed
			toggle_flywheel = !toggle_flywheel;
		}

		// Sets drivetrain speed in % (capped at 95%)
		drive_forward = controller.get_analog(ANALOG_LEFT_Y);
		drive_turn = controller.get_analog(ANALOG_RIGHT_X);

		left_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant + drive_turn);
		right_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant - drive_turn);

		left_drivetrain = left_drive_speed;
		right_drivetrain = right_drive_speed;

		loopRate.delay(okapi::QFrequency(50.0)); // basically a perfectly even 20 ms between starts of each iteration
	}
}

void ForwardPID(float target, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
{

	tracking_wheel_vertical.reset_position();
	int sensor;

	float error = target - tracking_wheel_vertical.get_position() / 360 * (2.75 * 3.14159);
	float prev_error;
	float integral;
	float derivative;
	int output = 0;

	float settle_distance = 2; // change this value to change what error the PID considers "settled"

	int timer = 0;
	int settle_timer = 0;

	if (timeout_msec = -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = error * 30 + 500; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	while (timer <= timeout_msec && settle_timer <= settle_time_msec)
	{

		sensor = tracking_wheel_vertical.get_position();

		// pros::lcd::print(2,"tracking wheel: %d",sensor);
		// pros::lcd::print(3,"error: %f",error);
		// pros::lcd::print(4,"output: %f",forward_kP*(error + forward_kI*integral + forward_kD*derivative));
		// pros::lcd::print(5,"output: %d",output);

		error = target - ((float)sensor / 36000 * 2.75 * 3.14159); // turns centidegrees into rotations, then turns rotations into inches travelled. We typecast sensor into a float because ints truncate, for example 50/100 = 0

		if (error < kI_start_at_error_value)
		{
			integral += error;
		}

		derivative = prev_error - error;

		prev_error = error;

		output = forward_kP * (error + forward_kI * integral + forward_kD * derivative);

		// cap output at 11000 milivolts (11 volts)
		if (output > 11000)
		{
			output = 11000;
		}

		// output to drivetrain
		left_drivetrain.move_voltage(output); // output needs to be between -12000 to 12000 milivolts
		right_drivetrain.move_voltage(output);

		if (abs(error) < settle_distance) // absolute value so it never goes negative
		{
			settle_timer += 20;
		}
		else
		{
			settle_timer = 0;
		}

		timer += 20;

		pros::delay(20);
	}
}

void TurnPID(float target, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
{
	float error = target - inertial_sensor.get_rotation(); // target is degrees we want to be at
	float prev_error;
	float integral;
	float derivative;
	float sensor;
	int output;

	float settle_distance = 1; // change this value to change what error the PID considers "settled"

	int timer = 0;
	int settle_timer = 0;

	if (timeout_msec = -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = error * 30 + 500; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	while (timer < timeout_msec && settle_timer < settle_time_msec)
	{
		sensor = inertial_sensor.get_rotation();

		error = target - sensor;

		if (error < kI_start_at_error_value)
		{
			integral += error;
		}

		derivative = prev_error - error;

		prev_error = error;

		output = turn_kP * (error + turn_kI * integral + turn_kD * derivative);

		// cap output at 11000 milivolts (11 volts)
		if (output > 11000)
		{
			output = 11000;
		}

		// output to drivetrain
		left_drivetrain.move_voltage(output); // output needs to be -12000 to 12000 milivolts
		right_drivetrain.move_voltage(-output);


		if (abs(error) < settle_distance) // absolute value so it never goes negative
		{
			settle_timer += 20;
		}
		else
		{
			settle_timer = 0;
		}

		timer += 20;

		pros::delay(20);
	}
}

void flywheel_bang_bang() // BANG BANG control
{
	int flywheel_counter = 0;
	int flywheel_rpm = 0;;
	while (true)
	{
		//get_velocity returns centidegrees per second, so we convert to rotations per minute
		flywheel_rpm = (float)flywheel_sensor.get_velocity() / 360 *60;

		if (toggle_flywheel)
		{
			if (flywheel_rpm < 3900)
			{
				flywheel_motor.move_voltage(11500);
			}
			else
			{
				flywheel_motor.move_voltage(0);
			}
		} else
		{
			flywheel_motor.brake();
		}
		// pros::lcd::print(2,"flywheel iteration #: %d",flywheel_counter);
		pros::lcd::print(3,"flywheel rpm: %d",flywheel_rpm);
		flywheel_counter++;
		pros::delay(20);
		//loopRate.delay(okapi::QFrequency(50.0)); // runs exactly 20ms between starts of each iteration
	}
}

void odometry()
{
	double x_wheel_position;
	double prev_x_wheel_position = 0;
	double y_wheel_position;
	double prev_y_wheel_position = 0;

	int odom_counter = 0;

	while(true)
	{
		// Absolute Angle //////////////////////////////////////////////////////////////
			robot_angle = inertial_sensor.get_rotation();
		/////////////////////////////////////////////////////////////

		// Absolute Position //////////////////////////////////////////////////////////////
			x_wheel_position = tracking_wheel_horizontal.get_position() / 360 * (2.75 * 3.14159);
			y_wheel_position = tracking_wheel_vertical.get_position() / 360 * (2.75 * 3.14159);

			x_arc = x_wheel_position - prev_x_wheel_position;
			y_arc = y_wheel_position - prev_y_wheel_position;

			robot_position[0] = 2*std::sin(robot_angle/2) * (x_arc/robot_angle + Tx); //x
			robot_position[1] = 2*std::sin(robot_angle/2) * (y_arc/robot_angle + Ty); //y
		

			prev_x_wheel_position = x_wheel_position;
			prev_y_wheel_position = y_wheel_position;
		/////////////////////////////////////////////////////////////

		pros::lcd::print(4,"position (x,y): (%lf,%lf)",robot_position[0],robot_position[1]);
		pros::lcd::print(5,"angle radians: %lf",robot_angle);
		pros::lcd::print(2,"odom_iterator: %d",odom_counter);


		odom_counter++;
		pros::delay(20);
	}
}


void goTo(float target_x, float target_y, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
{
	//Odom variables
	float relative_x; //inches
	float relative_y; //inches
	float error_distance; //inches
	float error_angle; //radians

	//PID variables for both distance and angle
	float prev_error_distance;
	float prev_error_angle;
	float integral;
	float derivative;
	int output_distance;
	int output_angle;

	float settle_distance = 2; // change this value to change what error the PID considers "settled"
	float settle_angle = 1;

	int timer = 0;
	int settle_timer = 0;

	if (timeout_msec = -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = error * 30 + 500; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	while(timer < timeout_msec && settle_timer < settle_time_msec) //give this an end condition
	{
		// Error from Odom /////////////////////////////////////////////////////////////////////////////
			relative_x = target_x - robot_position[0];
			relative_y = target_y - robot_position[1];

			error_distance = sqrt(pow(relative_x,2) + pow(relative_y,2));

/* ASK COACH */			error_angle = atan(relative_y / relative_x); //HOOOOOLD UP, this doesn't reference robot_angle at all, but (at least at 10:00 PM) it seems to work mathematically because the position is already relative to the robot (with the robot centered on the origin), and this is basical precalculus
		////////////////////////////////////////////////////////////////////////////

		
		// Distance PID /////////////////////////////////////////////////////////////////////////////
			if (error_distance < kI_start_at_error_value)
			{
				integral += error_distance;
			}

			derivative = prev_error_distance - error_distance;

			prev_error_distance = error_distance;

			output_distance = forward_kP * (error_distance + forward_kI * integral + forward_kD * derivative);
		////////////////////////////////////////////////////////////////////////////


		// Angle PID /////////////////////////////////////////////////////////////////////////////
			if (error_angle < kI_start_at_error_value)
			{
				integral += error_angle;
			}

			derivative = prev_error_angle - error_angle;

			prev_error_angle = error_angle;

			output_angle = turn_kP * (error_angle + turn_kI * integral + turn_kD * derivative);
			////////////////////////////////////////////////////////////////////////////


			////reduce output_distance if error_angle is too large? Coach mentioned using cos(error_angle) or something like that
		

		// cap output at 11000 milivolts (11 volts) ... hard cap or scale instead?
		if (output > 11000)
		{
			output = 11000;
		}

		// output to drivetrain
		left_drivetrain.move_voltage(output_distance + output_angle); // output needs to be -12000 to 12000 milivolts
		right_drivetrain.move_voltage(output_distance - output_angle); 


		if (abs(error_distance) < settle_distance && abs(error_angle) < settle_angle) // absolute value so it never goes negative
		{
			settle_timer += 20;
		}
		else
		{
			settle_timer = 0;
		}

		timer += 20;

		pros::delay(20);

	}

}