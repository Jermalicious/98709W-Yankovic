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
pros::Motor left_top_drive(13, true);
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

//passes in distance to a goal and returns a motor value between -11500 and 11500 milivolts
//returns 12001 when successfully finished movement
//returns 12002 when movement times out
//returns 12003 if unexpected failure
void driveTo(float target_x, float target_y, float settle_time_msec = 450, float settle_distance = 2.2, float kI_start_at_angle = 16, float kI_start_at_distance = 7, int timeout_msec = -1);
void turnTo(float target, float settle_time_msec = 400, float kI_start_at_error_value = 16, int timeout_msec = -1);

double reduce_angle_negative_180_to_180(double angle);

// declare global variables
const double forward_kP = 650;	//650
const double forward_kI = 0;	//0
const double forward_kD = 250;	//0 //450 new

const double turn_kP = 200;	//200
const double turn_kI = 16;	//16
const double turn_kD = 750;	//750

const double driveTo_turn_kP = 250; //200
const double driveTo_turn_kI = 0;   //16
const double driveTo_turn_kD = 300; //600

float starting_angle = 0; //angle of robot at start of auton, matters if we start at an odd angle but want to give field-relative instructions
float starting_x;
float starting_y;

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

	float θ1; //absolute angle in radians

	float Δdx; //change in x according to the field
	float Δdy; //change in y according to the field
	float Δdlx = 0; //change in x according to the robot
	float Δdly = 0; //change in y according to the robot
	float Δdl_polar_r;
	float Δdl_polar_θ;

	float y_offset = -2.5; //initialize //"SR" in odom paper, offset from vertical tracking wheel to tracking center in inches
	float x_offset = 0.25; //"SS" in odom paper, offset from horizontal tracking wheel to tracking center in inches. It's negative because left is -x direction 

	float Rr = 0;  //at the "last reset" in this case the beginning
	float Sr = 0;  //at the "last reset" in this case the beginning
	float θr = 0;  //at the "last reset" in this case the beginning

	float θm; //average orientation used for converting to field coordinates

	float d0x = 0; //previous robot position
	float d0y = 0; //previous robot position
	float θ0;      //previous robot angle
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
			absolute_robot_angle = reduce_angle_negative_180_to_180(inertial_sensor.get_rotation() * 1.0085 + starting_angle); //degrees

			θ1 = absolute_robot_angle / 180 * 3.14159; //radians

			Δθ = (θ1 - θ0);   //radians
			θm = θ0 + (Δθ/2); //radians
		/////////////////////////////////////////////////////////////

		// Absolute Position //////////////////////////////////////////////////////////////
			ΔS = (float)(tracking_wheel_horizontal.get_position() - prev_tracking_wheel_horizontal)/ 100 / 360 * (2.75 * 3.14159);   //WE HAVE A PROBLEM TRACKING X SOME
			ΔR = (float)(tracking_wheel_vertical.get_position() - prev_tracking_wheel_vertical)/ 100 / 360 * (2.75 * 3.14159);

			prev_tracking_wheel_horizontal = tracking_wheel_horizontal.get_position();
			prev_tracking_wheel_vertical = tracking_wheel_vertical.get_position();

			
			if (Δθ == 0)
			{
				// pros::lcd::print(1,"delta_S: %f", ΔS);
				
				Δdlx = ΔS;
				Δdly = ΔR;

				// pros::lcd::print(2,"delta_dlx: %f",Δdlx);
			} else
			{
				// pros::lcd::print(1,"delta_S: %f", ΔS);

				Δdlx = 2*sin((Δθ/2)) * (ΔS/Δθ + x_offset); //x       just adding the offset makes me feel suspicious about infinite slide when sin(Δθ / 2) != 0
				Δdly = 2*sin((Δθ/2)) * (ΔR/Δθ + y_offset); //y		std::sin and std::cos use RADIANS

				// pros::lcd::print(2,"delta_dlx: %f",Δdlx);

			}
///////////////////Probably OK to here

			Δdl_polar_r = sqrt(pow(Δdlx,2) + pow(Δdly,2)); //this gets rid of negatives, but that's because you can't HAVE negatives as a distance


			Δdl_polar_θ = atan2f(Δdly, Δdlx); //polar angle to be rotated by -θm = global polar angle CHECK here

			Δdx = Δdl_polar_r * cos(Δdl_polar_θ + (-θm)); //x polar angle rotated by -θm = global polar angle
			Δdy = Δdl_polar_r * sin(Δdl_polar_θ + (-θm)); //y

			robot_positionx = d0x + Δdx + starting_x; //x inches
			robot_positiony = d0y + Δdy + starting_y; //y inches


			θ0 = θ1; //previous angle = new angle

			d0x = robot_positionx - starting_x; //previous x = new x
			d0y = robot_positiony - starting_y; //previous y = new y


		
		/////////////////////////////////////////////////////////////

		pros::lcd::print(0,"position (x,y): (%f,%f)",robot_positionx,robot_positiony);
		pros::lcd::print(1,"absolute angle degeres: %f",absolute_robot_angle);
		// pros::lcd::print(3,"Δd: [%f , %f]", Δdx, Δdy);
		// pros::lcd::print(4,"Δdl: [%f , %f]", Δdlx, Δdly);
		// pros::lcd::print(6,"x_track: %d", tracking_wheel_horizontal.get_angle());
		// pros::lcd::print(7,"dl_pol_theta, %f , r: %f", Δdl_polar_θ / 3.14159 * 180, Δdl_polar_r);

		

		odom_counter++;
		pros::delay(15);
	}
}


void driveTo(float target_x, float target_y, float settle_time_msec, float settle_distance, float kI_start_at_angle, float kI_start_at_distance, int timeout_msec)
{
	//Odom variables
	float relative_x = target_x - robot_positionx; //inches
	float relative_y = target_y - robot_positiony; //inches
	float relative_angle = atan2f(relative_x, relative_y) / 3.1415 * 180;
	float error_distance = sqrt(pow(relative_x,2) + pow(relative_y,2)); //inches
	float error_angle = reduce_angle_negative_180_to_180(relative_angle - absolute_robot_angle); //degrees

	//PID variables for both distance and angle
	float prev_error_distance;
	float prev_error_angle;
	float integral;
	float derivative;
	int output_distance;
	int output_angle;
	int timer = 0;
	int settle_timer = 0;

	int distance_voltage_cap = 8000;
	int angle_voltage_cap = 5000;
	float heading_correction_factor;

	if (timeout_msec = -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = abs(error_angle) * 20 + abs(error_distance) * 40 + 1000; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	while(timer < timeout_msec && settle_timer < settle_time_msec)
	{
		// Error from Odom /////////////////////////////////////////////////////////////////////////////
			relative_x = target_x - robot_positionx;
			relative_y = target_y - robot_positiony;
			relative_angle = atan2f(relative_x, relative_y) / 3.1415 * 180;

			error_distance = sqrt(pow(relative_x,2) + pow(relative_y,2));
			error_angle = reduce_angle_negative_180_to_180(relative_angle - absolute_robot_angle); //degrees //degrees
		////////////////////////////////////////////////////////////////////////////
		
		// Distance PID /////////////////////////////////////////////////////////////////////////////
			if (error_distance < kI_start_at_distance)
			{
				integral += error_distance;
			}
			derivative = prev_error_distance - error_distance;
			prev_error_distance = error_distance;
			output_distance = forward_kP * error_distance + forward_kI * integral - forward_kD * derivative;
		////////////////////////////////////////////////////////////////////////////
		// Angle PID /////////////////////////////////////////////////////////////////////////////
			if (error_angle < kI_start_at_angle)
			{
				integral += error_angle;
			}
			derivative = prev_error_angle - error_angle;
			prev_error_angle = error_angle;
			output_angle = driveTo_turn_kP * error_angle + driveTo_turn_kI * integral - driveTo_turn_kD * derivative;
		////////////////////////////////////////////////////////////////////////////
			////reduce output_distance if error_angle is too large? Coach mentioned using cos(error_angle) or something like that
			heading_correction_factor = cos(error_angle * 3.1415 / 180);
			output_distance *= heading_correction_factor;
		// cap output at 11000 milivolts (11 volts) ... scales linearly to cap max volts at 11 but keep same ratio of power to keep turning
		// if (output_distance > 11000 && output_distance > output_angle)
		// {
		// 	output_angle /= (output_distance/11000);
		// 	output_distance /= (output_distance / 11000);
		// }
		// else if (output_angle > 11000 && output_angle > output_distance)
		// {
		// 	output_angle /= (output_angle/11000);
		// 	output_distance /= (output_angle / 11000);
		// }

		if (error_distance < settle_distance)
		{
			output_angle = 0;
		}

		if(output_distance > abs(distance_voltage_cap * heading_correction_factor))
		{
			output_distance = abs(distance_voltage_cap * heading_correction_factor);
		} else if (output_distance < -abs(distance_voltage_cap * heading_correction_factor))
		{
			output_distance = -abs(distance_voltage_cap * heading_correction_factor);
		}

		if(output_angle > angle_voltage_cap)
		{
			output_angle = angle_voltage_cap;
		}else if (output_angle < -angle_voltage_cap)
		{
			output_angle = -angle_voltage_cap;
		}

		// output to drivetrain
		left_drivetrain.move_voltage(output_distance + output_angle); // output needs to be -12000 to 12000 milivolts
		right_drivetrain.move_voltage(output_distance - output_angle);

		if (abs(error_distance) < settle_distance) // absolute value so it never goes negative
		{
			settle_timer += 20;
		}
		else
		{
			settle_timer = 0;
		}
		timer += 20;
		pros::delay(20);


		// controller.clear();
		controller.print(0,0,"error_dist: %f", error_distance);
		controller.print(1,0,"error_angle: %f", error_angle);

		pros::lcd::print(2,"error_distance: %f", error_distance);
		pros::lcd::print(3,"error_angle: %f", error_angle);
		pros::lcd::print(4,"timer: %d", timer);
		pros::lcd::print(5,"relative_angle: %f", relative_angle);
		// pros::lcd::print(5,"loop?: %d", timer < timeout_msec && settle_timer < settle_time_msec);
		// pros::lcd::print(6,"timeout active?: %d", timer < timeout_msec);
		// pros::lcd::print(7,"settle active?: %d", settle_timer < settle_time_msec);
		
	}

	left_drivetrain.brake();
	right_drivetrain.brake();
}

//1000 millivolts = 1 volt
//1000 milliseconds = 1 second
void drive_by_voltage(int millivolts, float milliseconds)
{
	left_drivetrain.move_voltage(millivolts);
	right_drivetrain.move_voltage(millivolts);

	pros::delay(milliseconds);

	left_drivetrain.brake();
	right_drivetrain.brake();
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

	pros::Task odometry_task(odometry);
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
		// driveTo(0,24);
		// goTo(0,24);
		// goTo(24,24);
		driveTo(72,24);
		driveTo(72,72);
		driveTo(96,24);
		driveTo(0,0);
		// turnTo(180);
		// turnTo(45);
		// turnTo(-70);
		// turnTo(0);

		// driveTo(0,0);
		turnTo(0);

		// ForwardPID(8); // push ball
		// intake = -95;  //outtake preload
		// pros::delay(1000);
		// intake.brake();

	} else if (auton_picker == 1) //skills
	{
		starting_angle = 0;
		starting_x = 24;
		starting_y = 0;

	//push preloads under the goal
		turnTo(-40);
		driveTo(4,28); 	//move to goal
		intake = -95;
		turnTo(0);		//orient toward goal
		drive_by_voltage(11000,700);	//push under the goal

	//drive to launch zone
		flywheel_motor.move_voltage(11000);	//spin up the flywheel
		driveTo(6,24);	//go to the LZ
		intake = 0;
		turnTo(60);		//turn toward target
		drive_by_voltage(-4500,500);
		pros::delay(35000);	//pause for chance to launch
		flywheel_motor.move_voltage(0);	//turn off flywheel

	//drive to first push
		driveTo(24,8,20,5);	//drive to entrance of the hallway
		driveTo(96,9,20,5);	//drive to other side through the hallway
		driveTo(116,30);
		turnTo(0);
		
	//push triballs under goal
		wings.set_value(HIGH);	//extend wings for the push
		pros::delay(400);	//wait for wings to deploy
		drive_by_voltage(11000,750);	//push under goal
		wings.set_value(LOW);
		drive_by_voltage(-9000,500);

	//drive to second push goal
		driveTo(96,30,20,5);
		// driveTo(74,30,20,5); //RISKY SWEEP
		driveTo(73,68);	//drive to a point in front of the goal
		turnTo(90);	//turn toward goal

	//push triablls under goal
		wings.set_value(HIGH);	//extend wings for the push
		pros::delay(400);	//wait for wings to deploy
		drive_by_voltage(11000,750);	//push under goal
		wings.set_value(LOW);
		drive_by_voltage(-9000,500);	//get out of the way, make sure we're not contacting any triballs

	} else if (auton_picker = 2) //shove ball under
	{	



	} else if (auton_picker = 3) //Clear Match Loads
	{
		wings.set_value(HIGH);
		pros::delay(1000); // 1 second

		// turnPID(180);
		pros::delay(1000);

		wings.set_value(LOW);
	}
	

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
	float drive_forward;
	float drive_turn;

	float left_drive_speed;
	float right_drive_speed;

	float const drive_turn_constant = 1.4;

	// initialize variables

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////Control Loop  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
				flywheel_motor.move_voltage(11000);
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

int forwardPID(float error, bool new_movement, float settle_time_msec, float kI_start_at_error_value, int timeout_msec) //FIX HOW TIMEOUT IS CALCULATED!
{

	static float prev_error = 0;
	static float integral = 0;
	static int timer = 0;
	static int settle_timer = 0;

	if (new_movement) 	//since static variables stick around between function calls, we need to reset these values...
	{					//...when we start a new movement so it doesn't "remember" the previous movement
		prev_error = 0;
		integral = 0;
		timer = 0;
		settle_timer = 0;
	}

	int sensor = tracking_wheel_vertical.get_position();

	float derivative;
	int output = 0;

	float settle_distance = 2; // change this value to change what error the PID considers "settled"




	if (timeout_msec == -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = abs(error) * 30 + 5000; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	if (timer <= timeout_msec && settle_timer <= settle_time_msec)
	{
		// pros::lcd::print(2,"tracking wheel: %d",sensor);
		// pros::lcd::print(3,"error: %f",error);
		// pros::lcd::print(4,"output: %f",forward_kP*(error + forward_kI*integral + forward_kD*derivative));
		// pros::lcd::print(5,"output: %d",output);


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

		if (abs(error) < settle_distance) // absolute value so it never goes negative
		{
			settle_timer += 20;
		}
		else
		{
			settle_timer = 0;
		}

		timer += 20;

		return(output);
	}

	if(settle_timer > settle_time_msec) //successfully finished movement
	{
		return(12001);
	} else if (timer > timeout_msec) //timed out
	{
		return(12002);
	} else //if this is called, something werid happened and you need to check it out
	{
		return(12003);
		pros::lcd::print(0,"FORWARD_PID FAILURE");
	}
}

int turnPID(float target, bool new_movement, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
{
	pros::lcd::print(0,"entered turnPID");

	static float prev_error = 0;
	static float integral = 0;
	static int timer = 0;
	static int settle_timer = 0;

	if (new_movement) 	//since static variables stick around between function calls, we need to reset these values...
	{					//...when we start a new movement so it doesn't "remember" the previous movement
		prev_error = 0;
		integral = 0;
		timer = 0;
		settle_timer = 0;
	}

	float error = target - (inertial_sensor.get_rotation()); // target is degrees we want to be at
	float derivative;
	float sensor;
	int output;

	float settle_distance = 3; // change this value to change what error the PID considers "settled"

	if (timeout_msec == -1) // this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = abs(error) * 30 + 5000; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	if (timer < timeout_msec && settle_timer < settle_time_msec)
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

		return(output);
	}

	if(settle_timer > settle_time_msec) //successfully finished movement
	{
		return(12001);
	} else if (timer > timeout_msec) //timed out
	{
		return(12002);
	} else //if this is called, something werid happened and you need to check it out
	{
		return(12003);
		pros::lcd::print(0,"FORWARD_PID FAILURE");
	}
}

void turnTo(float target, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
{
	pros::lcd::print(0,"entered turnPID");
	float error = reduce_angle_negative_180_to_180(target - (inertial_sensor.get_rotation() + starting_angle)); // target is degrees we want to be at
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
		error = reduce_angle_negative_180_to_180(target - (sensor + starting_angle));
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
		if (output > 11000)
		{
			output = 11000;
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
	left_drivetrain.move_voltage(0);
	right_drivetrain.move_voltage(0);
}


double reduce_angle_negative_180_to_180(double angle_degrees)
{
	// pros::lcd::print(5,"reduce angle: %f",angle_degrees);

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