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
pros::Rotation tracking_wheel_horizontal(1, false);
pros::Rotation tracking_wheel_vertical(5, false);
pros::Imu inertial_sensor(20);
pros::Rotation flywheel_sensor(7, true);

// define drivetrain motors
pros::Motor left_top_drive(12, true);
pros::Motor left_back_drive(3, false);
pros::Motor left_front_drive(4, true);
pros::Motor right_top_drive(10, false);
pros::Motor right_back_drive(9, true);
pros::Motor right_front_drive(8, false);

// define drivetrain motor groups
pros::Motor_Group left_drivetrain({left_top_drive, left_back_drive, left_front_drive});		// the three motors for the left side of the drivetrain
pros::Motor_Group right_drivetrain({right_top_drive, right_back_drive, right_front_drive}); // the three motors for the right side of the drivetrain
pros::Motor_Group intake({left_intake, right_intake});										// both intake motors

// declare functions so that we can define them at the bottom of this page
void ForwardPID(float target, float settle_time_msec = 500, float kI_start_at_error_value = 7, int timeout_msec = -1);
void TurnPID(float target, float settle_time_msec = 800, float kI_start_at_error_value = 16, int timeout_msec = -1);
double reduce_angle_negative_180_to_180(double angle);

// declare global variables
const double forward_kP = 650;
const double forward_kI = 0;
const double forward_kD = 0;

const double turn_kP = 200;
const double turn_kI = 16;
const double turn_kD = 600;

float starting_angle = 0; //angle of robot at start of auton, matters if we start at an odd angle but want to give field-relative instructions

int toggle_flywheel = 0;
int auton_picker = 0;

float robot_positionx = 0; //robot x position in inches
float robot_positiony = 0; //robot y position in inches
float absolute_robot_angle; //"θ1" in odom paper, but in degrees here


/*
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
		pros::lcd::set_text(6, "Skills");
		auton_picker = 1;
	}
	else
	{
		pros::lcd::clear_line(6);
		auton_picker = 1;
	}
}

void on_left_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(6, "shove ball under");
		auton_picker = 2;
	}
	else
	{
		pros::lcd::clear_line(6);
		auton_picker = 2;
	}
}

void on_right_button()
{
	pros::lcd::clear_line(6);
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(6, "clear Match Loads (Loading Zone triballs)");
		auton_picker = 3;
	}
	else
	{
		pros::lcd::clear_line(6);
		auton_picker = 3;
	}
}

void odometry()
{
	tracking_wheel_horizontal.set_position(0);
	tracking_wheel_vertical.set_position(0);
	int odom_counter = 0;

	float θ1;

	float Δdx;
	float Δdy;
	float Δdlx = 0;
	float Δdly = 0;
	float Δdl_polar_r;
	float Δdl_polar_θ;

	float y_offset = -2.5; //initialize //"SR" in odom paper, offset from vertical tracking wheel to tracking center in inches
	float x_offset = 0.25; //"SS" in odom paper, offset from horizontal tracking wheel to tracking center in inches. It's negative because left is -x direction 

	float Rr = 0; //at the "last reset" in this case the beginning
	float Sr = 0; //at the "last reset" in this case the beginning
	float θr = 0;  //at the "last reset" in this case the beginning

	float θm; //average orientation used for converting to field coordinates

	float d0x = 0; //previous robot position
	float d0y = 0; //previous robot position
	float θ0; //previous robot angle
	float prev_tracking_wheel_horizontal;
	float prev_tracking_wheel_vertical;

	float ΔS = tracking_wheel_vertical.get_position() - prev_tracking_wheel_vertical;
	float ΔR = tracking_wheel_horizontal.get_position() - prev_tracking_wheel_horizontal;
	float Δθ; //radians
	

	//double ΔRr = tracking_wheel_vertical.get_position() - Rr;   //only needed to find angle
	//double ΔLr = tracking_wheel_horizontal.get_position() - Sr; //only needed to find angle



	int calibrate_timer = 0;

	inertial_sensor.reset();
	while(inertial_sensor.is_calibrating())
	{
		pros::lcd::print(1, "inertial taken %f seconds to calibrate", (float)calibrate_timer/1000);
		calibrate_timer += 20;
		pros::delay(20);

	}

	while(true)
	{
		// Absolute Angle //////////////////////////////////////////////////////////////
			absolute_robot_angle = reduce_angle_negative_180_to_180(inertial_sensor.get_rotation()); //degrees

			θ1 = absolute_robot_angle / 180 * 3.14159; //radians

			Δθ = (θ1 - θ0); //radians
			θm = θ0 + (Δθ/2); //radians
		/////////////////////////////////////////////////////////////

		// Absolute Position //////////////////////////////////////////////////////////////
			ΔS = (float)(tracking_wheel_horizontal.get_position() - prev_tracking_wheel_horizontal)/ 100 / 360 * (2.75 * 3.14159);
			ΔR = (float)(tracking_wheel_vertical.get_position() - prev_tracking_wheel_vertical)/ 100 / 360 * (2.75 * 3.14159);

			prev_tracking_wheel_horizontal = tracking_wheel_horizontal.get_position();
			prev_tracking_wheel_vertical = tracking_wheel_vertical.get_position();

			
			if (Δθ == 0)
			{
				pros::lcd::print(1,"delta_S: %f", ΔS);
				
				Δdlx = ΔS;
				Δdly = ΔR;

				pros::lcd::print(2,"delta_dlx: %f",Δdlx);
			} else
			{
				pros::lcd::print(1,"delta_S: %f", ΔS);

				Δdlx = 2*sin((Δθ/2)) *(ΔS/Δθ + x_offset); //x       just adding the offset makes me feel suspicious about infinite slide when sin(Δθ / 2) != 0
				Δdly = 2*sin((Δθ/2)) * (ΔR/Δθ + y_offset); //y		std::sin and std::cos use RADIANS

				pros::lcd::print(2,"delta_dlx: %f",Δdlx);

			}
///////////////////Probably OK to here

			Δdl_polar_r = sqrt(pow(Δdlx,2) + pow(Δdly,2)); //this gets rid of negatives, but that's because you can't HAVE negatices as a distance


			Δdl_polar_θ = atan2f(Δdly, Δdlx); //polar angle rotated by -θm = global polar angle CHECK here

			Δdx = Δdl_polar_r * cos(Δdl_polar_θ + (-θm)); //x polar angle rotated by -θm = global polar angle
			Δdy = Δdl_polar_r * sin(Δdl_polar_θ + (-θm)); //y

			robot_positionx = d0x + Δdx; //x inches
			robot_positiony = d0y + Δdy; //y inches


			θ0 = θ1; //previous angle = new angle

			d0x = robot_positionx; //previous x = new x
			d0y = robot_positiony; //previous y = new y


		
		/////////////////////////////////////////////////////////////

		pros::lcd::print(1,"position (x,y): (%f,%f)",robot_positionx,robot_positiony);
		pros::lcd::print(5,"absolute angle degeres: %f",absolute_robot_angle);
		pros::lcd::print(3,"Δd: [%f , %f]", Δdx, Δdy);
		pros::lcd::print(4,"Δdl: [%f , %f]", Δdlx, Δdly);
		pros::lcd::print(6,"x_track: %d", tracking_wheel_horizontal.get_angle());
		pros::lcd::print(7,"dl_pol_theta, %f , r: %f", Δdl_polar_θ / 3.14159 * 180, Δdl_polar_r);

		

		odom_counter++;
		pros::delay(20);
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
	// pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::set_text(7, "Shove Under  Skills     Clear ML");
	pros::lcd::set_text(6, "go forward then outtake");

	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
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
 * an external microSD card constantly.
 */

void autonomous()
{
	while(inertial_sensor.is_calibrating())
	{
		pros::delay(20);
	}

	if(auton_picker == 0) //go forward and outtake
	{
		left_top_drive.move_voltage(11000); // left top is only moving forna little bit. Test wire and motor viability!
		
		// ForwardPID(8); // push ball
		// intake = -95;  //outtake preload
		// pros::delay(1000);
		// intake.brake();

	} else if (auton_picker == 1) //skills
	{
		starting_angle = 60;
		ForwardPID(-30.);

		flywheel_motor.move_voltage(11900);

		ForwardPID(9.5);
		
		TurnPID(-20);
		left_drivetrain.move_voltage(-8000);
		right_drivetrain.move_voltage(-8000);
		pros::delay(1000);
		left_drivetrain.brake();
		right_drivetrain.brake();

	} else if (auton_picker = 2) //shove ball under
	{	

		ForwardPID(-30);
		ForwardPID(6);

	} else if (auton_picker = 3) //Clear Match Loads
	{
		wings.set_value(HIGH);
		pros::delay(1000); // 1 second

		TurnPID(180);
		pros::delay(1000);

		wings.set_value(LOW);
	}
	
	

	// ForwardPID(24);
	// ForwardPID(-24);

	// left_drivetrain = -50; //turn left (front of robot = forward)
	// right_drivetrain = 50;
	// pros::delay (200); // wait .2 sec
	// left_drivetrain = 0;
	// right_drivetrain = 0;

	// TurnPID(180);
	// TurnPID(45);
	// ForwardPID(24);
	// TurnPID(0);


	// PRE-MATCH AUTON:
// ForwardPID(8); // push ball
// intake = -95;  //outtake preload
// intake = 0;
// ForwardPID(-36); // go backwards
// TurnPID(-135);
// ForwardPID(-48); // push team ball behind to the loading ground bar
// TurnPID(-180);
// ForwardPID(-24); // push team ball into goal

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
	// pros::Task flywheel_task(flywheel_bang_bang);
	// pros::Task print_test(print_task_test);

	// pros::Task odometry(track_position);

	// decalre variables

	float drive_forward;
	float drive_turn;

	float left_drive_speed;
	float right_drive_speed;

	float const drive_turn_constant = 1.4;

	// initialize variables

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////Control Loop/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	while (true)
	{

		// printing on the brain screen
		//  pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                   (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                   (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// intake controller
		if (controller.get_digital(DIGITAL_R1)) // forward
		{
			intake = 95;
		}
		else if (controller.get_digital(DIGITAL_L1)) // reverse
		{
			intake = -95;
		}
		else
		{
			intake.brake();
		}

		// wings controller
		if (controller.get_digital(DIGITAL_B))
		{
			wings.set_value(HIGH);
		}
		else
		{
			wings.set_value(LOW);
		}

		// turns flywheel on and off
		if (controller.get_digital_new_press(DIGITAL_R2))
		{ // function to toggle flywheel forward when R2 is pressed
			if (!toggle_flywheel)
			{
				flywheel_motor.move_voltage(10000);
				toggle_flywheel = 1;
			}
			else if (toggle_flywheel)
			{
				flywheel_motor.move_voltage(0);
				toggle_flywheel = 0;
			}

			pros::lcd::print(4,"Forward!");
		}

		if (controller.get_digital_new_press(DIGITAL_L2))
		{ // function to toggle flywheel reverse when L2 is pressed
			if (!toggle_flywheel)
			{
				flywheel_motor.move_voltage(-11900);
				toggle_flywheel = 1;
			}
			else if (toggle_flywheel)
			{
				flywheel_motor.move_voltage(0);
				toggle_flywheel = 0;
			}
			pros::lcd::print(4,"Reverse!");
		}

		// Sets drivetrain speed in % (capped at 95%)
		drive_forward = controller.get_analog(ANALOG_LEFT_Y);
		drive_turn = controller.get_analog(ANALOG_RIGHT_X);

		left_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant + drive_turn);
		right_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant - drive_turn);

		left_drivetrain = left_drive_speed;
		right_drivetrain = right_drive_speed;

		// loopRate.delay(okapi::QFrequency(50.0)); //breaks stuff in tasks. Avoid. // basically a perfectly even 20 ms between starts of each iteration
		pros::delay(20);
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

	if (timeout_msec == -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = abs(error) * 30 + 5000; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
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

		output = forward_kP * error + forward_kI * integral - forward_kD * derivative;

		// cap output at 11500 milivolts (11.5 volts)
		if (output > 11500)
		{
			output = 11500;
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
	pros::lcd::print(0,"entered turnPID");

	float error = target - (inertial_sensor.get_rotation() + starting_angle); // target is degrees we want to be at
	float prev_error;
	float integral;
	float derivative;
	float sensor;
	int output;

	float settle_distance = 3; // change this value to change what error the PID considers "settled"

	int timer = 0;
	int settle_timer = 0;

	if (timeout_msec == -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = abs(error) * 30 + 5000; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	while (timer < timeout_msec && settle_timer < settle_time_msec)
	{
		sensor = inertial_sensor.get_rotation();

		error = target - (sensor + starting_angle);

		// pros::lcd::print(3,"%f",reduce_angle_negative_180_to_180(370));

		if (abs(error) < kI_start_at_error_value)
		{
			integral += error;
		}
		else
		{
			integral = 0;
		}

		derivative = prev_error - error;

		prev_error = error;

		output = turn_kP * error + turn_kI * integral - turn_kD * derivative;

		// cap output at 11500 milivolts (11 volts)
		if (output > 11500)
		{
			output = 11500;
		}

		// output to drivetrain
		left_drivetrain.move_voltage(output); // output needs to be -12000 to 12000 milivolts
		right_drivetrain.move_voltage(-output);

		pros::lcd::print(4,"settle_timer: %d", settle_timer);
		pros::lcd::print(2,"integral: %f", integral);
		pros::lcd::print(3,"output: %d", output);

		if (abs(error) < settle_distance) // absolute value so it never goes negative
		{
			settle_timer += 20;
		}
		else
		{
			settle_timer = 0;
		}

		timer += 20;

		pros::lcd::print(1,"error_angle: %f", error);
		pros::lcd::print(5,"loop?: %d", timer < timeout_msec && settle_timer < settle_time_msec);
		pros::lcd::print(6,"timeout active?: %d", timer < timeout_msec);
		pros::lcd::print(7,"settle active?: %d", settle_timer < settle_time_msec);


		pros::delay(20);
	}

	left_drivetrain.move_voltage(0); // output needs to be -12000 to 12000 milivolts
	right_drivetrain.move_voltage(0);
}


double reduce_angle_negative_180_to_180(double angle_degrees)
{
	pros::lcd::print(5,"reduce angle: %f",angle_degrees);

	while(angle_degrees > 180)
	{
		angle_degrees -= 360;

		pros::lcd::print(6,"reduce angle(loop): %f",angle_degrees);
	}

	while(angle_degrees <= -180)
	{
		angle_degrees += 360;

		pros::lcd::print(6,"reduce angle(loop): %f",angle_degrees);
	}

	return(angle_degrees);
}