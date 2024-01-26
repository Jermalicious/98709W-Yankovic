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

// declare functions so that we can define them at the bottom of this page
void ForwardPID(float target, float settle_time_msec = 500, float kI_start_at_error_value = 7, int timeout_msec = -1);
void TurnPID(float target, float settle_time_msec = 500, float kI_start_at_error_value = 45, int timeout_msec = -1);
void flywheel_bang_bang();
void odometry();
void goTo(float target_x, float target_y, float settle_time_msec, float kI_start_at_angle = 45, float kI_start_at_distance = 7, int timeout_msec = -1);
void update_odom_sensors();

void track_position();

double reduce_angle_negative_180_to_180(double angle);

// declare global variables
const double forward_kP = 500;
const double forward_kI = 0;
const double forward_kD = 0;

const double turn_kP = 500;
const double turn_kI = .1;
const double turn_kD = 10;

bool toggle_flywheel = 0;


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
	// pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

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
	//Crude Pre-Match Auton
intake = 95;
left_drivetrain = 95; //intake ball under bar
right_drivetrain = 95;
pros::delay (200); //200 msec (.2 sec)
	intake = 0; 
	left_drivetrain = -95;//back up into corner ground ball
	right_drivetrain = -95;
	pros::delay (400); // wait 400 msec
left_drivetrain = 50; //turn left (front of robot = forward)
right_drivetrain = -50;
pros::delay (100); // wait 100 msec
	left_drivetrain = -95;//back up into wall
	right_drivetrain = -95;
	pros::delay (200);// wait 200 msec
left_drivetrain = 50; //turn left (front of robot = forward)
right_drivetrain = -50;
pros::delay (100); // wait 100 msec
	left_drivetrain = -95;//back up into wall
	right_drivetrain = -95; 
left_drivetrain = 0; //Stop moving
right_drivetrain = 0;


	// PRE-MATCH AUTON:
// intake = 95;
// ForwardPID(6); // intake ball under the bar
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
	// pros::Task odometry_task(odometry);
	// pros::Task flywheel_task(flywheel_bang_bang);
	// pros::Task print_test(print_task_test);

	pros::Task odometry(track_position);

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
		//  pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                   (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                   (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

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
		// pros::lcd::print(3,"flywheel rpm: %d",flywheel_rpm);
		flywheel_counter++;
		pros::delay(20);
		//loopRate.delay(okapi::QFrequency(50.0)); // runs exactly 20ms between starts of each iteration
	}
}

void odometry()
{
	tracking_wheel_horizontal.set_position(0);
	tracking_wheel_vertical.set_position(0);
	int odom_counter = 0;

	double absolute_robot_angle; //"θ1" in odom paper, but in degrees here
	double θ1;
	double robot_position[2] = {0,0}; //[ x , y ] in inches
	double Δd[2];
	double Δdl[2] = {0,0};
	double Δdl_polar[2]; //{ r, θ }

	double y_offset = 0.25; //initialize //"SR" in odom paper, offset from vertical tracking wheel to tracking center in inches
	double x_offset = -2.5; //"SS" in odom paper, offset from horizontal tracking wheel to tracking center in inches. It's negative because left is -x direction 
	double y_arc;
	double x_arc;

	double Rr = 0; //at the "last reset" in this case the beginning
	double Sr = 0; //at the "last reset" in this case the beginning
	double θr = 0;  //at the "last reset" in this case the beginning

	double θm; //average orientation used for converting to field coordinates

	double d0[2] = {0,0}; //previous robot position
	double θ0; //previous robot angle
	double prev_tracking_wheel_horizontal;
	double prev_tracking_wheel_vertical;

	double ΔS = tracking_wheel_vertical.get_position() - prev_tracking_wheel_vertical;
	double ΔR = tracking_wheel_horizontal.get_position() - prev_tracking_wheel_horizontal;
	double Δθ; //radians
	

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
				Δdl[0] = ΔS;
				Δdl[1] = ΔR;
			} else
			{
				Δdl[0] = 2*sin((Δθ/2)) *(ΔS/Δθ + x_offset); //x
				Δdl[1] = 2*sin((Δθ/2)) * (ΔR/Δθ + y_offset); //y
			}

			Δdl_polar[0] = sqrt(pow(Δdl[0],2) + pow(Δdl[1],2));
			Δdl_polar[1] = atan(Δdl[1] / Δdl[0]) + (-θm); //polar angle rotated by -θm = global polar angle

			Δd[0] = Δdl_polar[0] * std::cos(Δdl_polar[1]); //x
			Δd[1] = Δdl_polar[0] * std::sin(Δdl_polar[1]); //y

			robot_position[0] = d0[0] + Δd[0]; //x inches
			robot_position[1] = d0[1] + Δd[1]; //y inches


			θ0 = θ1; //previous angle = new angle

			d0[0] = robot_position[0]; //previous x = new x
			d0[1] = robot_position[1]; //previous y = new y


		
		/////////////////////////////////////////////////////////////

		pros::lcd::print(0,"position (x,y): (%f,%f)",robot_position[0],robot_position[1]);
		pros::lcd::print(1,"angle radians: %f",Δθ);
		pros::lcd::print(2,"absolute angle degeres: %f",absolute_robot_angle);
		pros::lcd::print(3,"Δd: [%f , %f]", Δd[0], Δd[1]);
		pros::lcd::print(4,"Δdl: [%f , %f]", Δdl[0], Δdl[1]);
		pros::lcd::print(5,"Δdl_polar: [%f , %f]", Δdl_polar[0], Δdl_polar[1]);
		pros::lcd::print(6,"y_pos: %f", Δd[1]);
		

		odom_counter++;
		pros::delay(1000);
	}
}


/*---------------------------------------//TRACKING SENSOR VARIABLES//------------------------------------------------------*/

float Forward_Tracker_Distance = 2.25;            //Distance from forward tracking wheel to robot center
float Sideways_Tracker_Distance = 3.875;            //Distance from sideways tracking wheel to robot center

float Forward_Tracker_Inches_per_Degree = 0.023617;  //Ratio used to convert degrees on tracking wheels to inches of movement 
float Sideways_Tracker_Inches_per_Degree = 0.023949;  //Use "Calibrate Trackers" program to calculate these values 

float Previous_Forward_Tracker_Reading = 0;       //Variables used to calculate the change in tracker readings
float Previous_Sideways_Tracker_Reading = 0;      //These changes will hereafter be referred to as "deltas"

float Delta_Forward_Tracker = 0;                  //Changes in the forward and sideways tracking wheels. These changes
float Delta_Sideways_Tracker = 0;                 //are used to calculate each movement and update global position

float Current_Forward_Tracker_Reading = 0;        //These variables are used to establish tracking wheel readings at a given
float Current_Sideways_Tracker_Reading = 0;       //time so that changes can be periodically noted.

/*-----------------------------------------//POSITIONING VARIABLES//--------------------------------------------------------*/

float local_X_position = 0;                       // Local position variables used to specify local movements which will then
float local_Y_position = 0;                       // be used as a reference for Global Position.

float absolute_global_X = 0;                      // Absolute position in terms of x and y coordinates
float absolute_global_Y = 0;

float previous_global_X = 0;                      // Variables used to store previous global positions. These will be used to 
float previous_global_Y = 0;                      // calculate changes in global position.

float delta_global_X = 0;                         // Change in absolute position in terms of x and y coordinates.  These values
float delta_global_Y = 0;                         // are recalculated with each loop of the "update_robot_position" function.

float local_polar_angle = 0;                      // Variables for robot position in polar coordinates.  Local angle and length
float local_polar_length = 0;                     // are used for each localized movement while global refers to the absolute
float global_polar_angle = 0;                     // Polar angle

float absolute_heading_degrees = 0;               // Absolute heading of the robot with the option of listing it in degrees
float absolute_heading_radians = 0;               // or radians.

float previous_heading_degrees = 0;               // Variables used to store previous heading in degrees or radians.
float previous_heading_radians = 0;

float delta_heading_degrees = 0;                  // Variables used to store the change in heading for every localized
float delta_heading_radians = 0;                  // movement in degrees or radians.




void update_tracking_sensors()
{
  // Set current tracker and heading measurements
  Current_Forward_Tracker_Reading = (float)tracking_wheel_vertical.get_position()/ 100 / 360 * (2.75 * 3.14159);
  Current_Sideways_Tracker_Reading = (float)tracking_wheel_horizontal.get_position()/ 100 / 360 * (2.75 * 3.14159);
//   absolute_heading_degrees = reduce_angle_negative_180_to_180(inertial_sensor.get_rotation());//*1.16 //1.16 represents a drift correction factor.
  absolute_heading_radians = absolute_heading_degrees / 180 * 3.14159;


  // Calculate change in tracker and heading measurements. Used to calculate local movements
  Delta_Forward_Tracker = Current_Forward_Tracker_Reading - Previous_Forward_Tracker_Reading;
  Delta_Sideways_Tracker = Current_Sideways_Tracker_Reading - Previous_Sideways_Tracker_Reading;
  delta_heading_degrees = absolute_heading_degrees-previous_heading_degrees;
  delta_heading_radians = absolute_heading_radians-previous_heading_radians;

  // Set current tracker readings and heading as "previous" values.  Will allow us to calculate
  // changes (deltas) in next loop.
  Previous_Forward_Tracker_Reading = Current_Forward_Tracker_Reading;
  Previous_Sideways_Tracker_Reading = Current_Sideways_Tracker_Reading;
  previous_heading_degrees = absolute_heading_degrees;
  previous_heading_radians = absolute_heading_radians;
}

void update_robot_position()
{
  // If the heading of the robot has not changed, local x and local y values will be directly equal to the distances measured by
  // the forward and sideways tracking wheels. 
  if(delta_heading_radians == 0)
  {
    local_X_position = Delta_Sideways_Tracker;
    local_Y_position = Delta_Forward_Tracker;
  }
  // If the angle has changed, then local x and y can be obtained by calculating the chord length of the arc intercepted by the
  // robot's movement.  The chord length is given by (2* sin(theta/2)*radius), where theta is the heading of the robot and the radius
  // is calculated by dividing the tracker wheel distance by the heading of the robot then adding the tracker wheel distance from
  // robot center. (Radius = Arc Length/Angle)
  else
  {
    local_X_position = (2*sin(delta_heading_radians/2))*((Delta_Sideways_Tracker/delta_heading_radians)+Sideways_Tracker_Distance);
    local_Y_position = (2*sin(delta_heading_radians/2))*((Delta_Forward_Tracker/delta_heading_radians)+Forward_Tracker_Distance);
  }

  // We convert local X and Y into polar coordinates.  If there are both zero (i.e. the robot hasn't moved), we immediately set local
  // polar angle and length to 0.
  if(local_X_position == 0 && local_Y_position == 0)
  {
    local_polar_angle = 0;
    local_polar_length = 0;
  }
  // Otherwise, local polar angle is found by taking the arctan of our position and local polar length is found by applying the 
  // distance equation for our x and y coordinates.
  else
  {
    local_polar_angle = atan2(local_Y_position,local_X_position);
    local_polar_length = sqrt(pow(local_X_position,2)+pow(local_Y_position,2));
  }

  // Global polar angle found by taking the local polar angle and subtracting the previous heading and then half of the change in 
  // heading.  This corrects for our shifted local coordinate system
  global_polar_angle = local_polar_angle - previous_heading_radians - (delta_heading_radians/2);

  // Change in global x and y can then be calculated by converting our local polar coordinates back to X and Y values
  delta_global_X = local_polar_length*cos(global_polar_angle);
  delta_global_Y = local_polar_length*sin(global_polar_angle);

  // Adding our change in global position to our previous global posiiton gives us our new absolute position in terms of x and
  // y coordinates.
  absolute_global_X = previous_global_X + delta_global_X;
  absolute_global_Y = previous_global_Y + delta_global_Y;

  // Set previous positions as our current absolute positions so that the next run can properly calculate changes.
  previous_global_X = absolute_global_X;
  previous_global_Y = absolute_global_Y;

  // Set previous heading as our current heading for the same reason.
  previous_heading_radians = absolute_heading_radians;
}

void track_position()
{
  while(true)
  {
    update_robot_position();
    update_tracking_sensors();
	

    //Print absolute x, absolute y, and heading on the brain screen
    pros::lcd::print(0, "Absolute X: %f Inches", absolute_global_X);
    pros::lcd::print(1, "Absolute Y: %f Inches", absolute_global_Y);
    pros::lcd::print(2, "Absolute Heading: %f Radians, %f Degrees", absolute_heading_radians, absolute_heading_degrees);
  }
}

/*
void goTo(float target_x, float target_y, float settle_time_msec, float kI_start_at_angle, float kI_start_at_distance, int timeout_msec)
{
	//Odom variables
	float relative_x; //inches
	float relative_y; //inches
	float error_distance; //inches
	float error_angle; //centidegrees

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
		timeout_msec = error_angle * 30 + error_distance * 30 + 1000; // sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}

	while(timer < timeout_msec && settle_timer < settle_time_msec) //give this an end condition
	{
		// Error from Odom /////////////////////////////////////////////////////////////////////////////
			relative_x = target_x - robot_position[0];
			relative_y = target_y - robot_position[1];

			error_distance = sqrt(pow(relative_x,2) + pow(relative_y,2));

		error_angle = atan(relative_y / relative_x) / 3.1415 * 180 - absolute_robot_angle; //centidegrees
		////////////////////////////////////////////////////////////////////////////

		
		// Distance PID /////////////////////////////////////////////////////////////////////////////
			if (error_distance < kI_start_at_distance)
			{
				integral += error_distance;
			}

			derivative = prev_error_distance - error_distance;

			prev_error_distance = error_distance;

			output_distance = forward_kP * (error_distance + forward_kI * integral + forward_kD * derivative);
		////////////////////////////////////////////////////////////////////////////


		// Angle PID /////////////////////////////////////////////////////////////////////////////
			if (error_angle < kI_start_at_angle)
			{
				integral += error_angle;
			}

			derivative = prev_error_angle - error_angle;

			prev_error_angle = error_angle;

			output_angle = turn_kP * (error_angle + turn_kI * integral + turn_kD * derivative);
		////////////////////////////////////////////////////////////////////////////


			////reduce output_distance if error_angle is too large? Coach mentioned using cos(error_angle) or something like that
		

		// cap output at 11000 milivolts (11 volts) ... scales linearly to cap max volts at 11 but keep same ratio of power to keep turning
		if (output_distance > 11000 && output_distance > output_angle)
		{
			output_angle /= (output_distance/11000);
			output_distance /= (output_distance / 11000);
		}
		else if (output_angle > 11000 && output_angle > output_distance)
		{
			output_angle /= (output_angle/11000);
			output_distance /= (output_angle / 11000);
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
*/

double reduce_angle_negative_180_to_180(double angle_degrees)
{
	pros::lcd::print(5,"reduce angle: %f",angle_degrees);

	while((angle_degrees > 180) || (angle_degrees <= -180))
	{
		if(angle_degrees > 180) 
		{
			angle_degrees -= 360;
		}
		else if(angle_degrees <= -180) 
		{
			angle_degrees += 360;
		}

		pros::lcd::print(6,"reduce angle(loop): %f",angle_degrees);
	}

	return(angle_degrees);
}

void update_odom_sensors()
{

}