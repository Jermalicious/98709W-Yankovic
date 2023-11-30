#include "main.h"
#include "classesTesting.h"
// #include "autonFunctions.h"
#include "okapi/api.hpp"

//define classes from classesTesting.h
	CustomMath customMath;
	okapi::Rate loopRate;

// define controller
	pros::Controller controller (pros::E_CONTROLLER_MASTER);

//define miscellaneous motors, pneumatics, and tracking wheels 
	pros::Motor flywheel_motor (15,true);		//Catapult mortor
	pros::Motor left_intake(19,true);		//the left intake motor
	pros::Motor right_intake(7,false); 		//the right intake motor
	pros::ADIDigitalOut wings (1,LOW); 		//the pneumatics to extend the pusher wings
	pros::Rotation tracking_wheel_X (13,false);	//Tracking wheels
	pros::Rotation tracking_wheel_Y	(12,false);
	pros::Imu inertial_sensor (14);

//define drivetrain motors
	pros::Motor left_top_drive (2,true);
	pros::Motor left_back_drive (3,false);
	pros::Motor left_front_drive (4, true);
	pros::Motor right_top_drive (10,false);
	pros::Motor	right_back_drive (9,true);
	pros::Motor right_front_drive (8,false);

//define drivetrain motor groups
	pros::Motor_Group left_drivetrain	({left_top_drive,	left_back_drive,	left_front_drive});  //the three motors for the left side of the drivetrain
	pros::Motor_Group right_drivetrain	({right_top_drive,	right_back_drive,	right_front_drive}); //the three motors for the right side of the drivetrain
	pros::Motor_Group intake			({left_intake,		right_intake}); 						 //both intake motors

//declare functions so taht we can define them at the bottom of this page
void ForwardPID	(float target, float settle_time_msec = 500, float kI_start_at_error_value = 7,  int timeout_msec = -1);
void TurnPID	(float target, float settle_time_msec = 500, float kI_start_at_error_value = 45, int timeout_msec = -1);
void flywheel_bang_bang ();

//declare global variables
const double forward_kP = 500;
const double forward_kI = 0;
const double forward_kD = 0;

const double turn_kP = 500;
const double turn_kI = .1;
const double turn_kD = 10;

bool toggle_a = 0;




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	while(inertial_sensor.is_calibrating())
	{
		pros::delay(20);
	}

	autonomous();
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
		// //Basic PID tuning routine
		// ForwardPID(12,500,6,50000);

		// TurnPID(180,500,45,50000);

		// ForwardPID(12,500,6,50000);

		// TurnPID(0,500,45,50000);

//PRE-MATCH AUTON:
		inertial_sensor.set_rotation(-90);
		intake = 95;
		ForwardPID (6);
		intake = 0;
		ForwardPID(-36);
		TurnPID(-135);
		ForwardPID(-48);
		TurnPID(-180);
		ForwardPID(-24);





//SKILLS RUN:		
		// for(int i = 0; i < 9; i++) //runs the fire routine 42 times (two extra for now)
		// {
		// 	cata_motor.move_relative(150 * 3, 99); //180 Degrees = 1 fire; 150 ticks (red motor encoder units) = 180 degrees; gear_ratio = 36:12 = 3:1
			
		// }

	//move to other zone

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


void opcontrol() {
//Set button function varialbe values
 flywheel_bang_bang();

//decalre variables

float drive_forward;
float drive_turn;

float left_drive_speed;
float right_drive_speed;

float const drive_turn_constant = 1.4;

//initialize variables

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Control Loop///////////////////////////////////;////////////////////////////;//////////////////;////////////////////////;////////////////////////////////////////////////////////////////////////////////////////////////////////////


	while (true){

//printing on the brain screen
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);


	//intake controller
		if (controller.get_digital(DIGITAL_L1)) //forward
		{ 
			intake = 95;
		} else if (controller.get_digital(DIGITAL_R1)) //reverse
		{ 
			intake = -95;
		} else 
		{
			intake.brake();
		}

	//wings controller
		if (controller.get_digital(DIGITAL_R2))
		{ 
			wings.set_value(HIGH);
		} else
		{
			wings.set_value(LOW);
		}

		if (controller.get_digital_new_press(DIGITAL_A)){ //function to toggle flywheel when A is pressed
			toggle_a = !toggle_a;
		}

	//Sets drivetrain speed in % (capped at 95%)
		drive_forward = controller.get_analog(ANALOG_LEFT_Y);
		drive_turn = controller.get_analog(ANALOG_RIGHT_X);

		left_drive_speed  = customMath.drive_cubic(drive_forward * drive_turn_constant + drive_turn);
		right_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant - drive_turn);

		left_drivetrain  = left_drive_speed;
		right_drivetrain = right_drive_speed;


		loopRate.delay(okapi::QFrequency(50.0)); //basically a perfectly even 20 ms between starts of each iteration
	
	}
}




void ForwardPID(float target, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
    {
	
	tracking_wheel_Y.reset_position();
	int sensor;
    
	float error = target - tracking_wheel_Y.get_position() / 360 * (2.75 * 3.14159);
    float prev_error;
    float integral;
    float derivative;
	int output = 0;

	float settle_distance = 2; //change this value to change what error the PID considers "settled"

	int timer = 0;
	int settle_timer = 0;

	if (timeout_msec = -1) //this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = error * 30 + 500; //sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}


    while(timer <= timeout_msec && settle_timer <= settle_time_msec)
    {
	
	sensor = tracking_wheel_Y.get_position();
	
	pros::lcd::print(2,"tracking wheel: %d",sensor);
	pros::lcd::print(3,"error: %f",error);
	pros::lcd::print(4,"output: %f",forward_kP*(error + forward_kI*integral + forward_kD*derivative));
	pros::lcd::print(5,"output: %d",output);

	error = target - ((float)sensor / 36000 * 2.75 * 3.14159); //turns centidegrees into rotations, then turns rotations into inches travelled. We typecast sensor into a float because ints truncate, for example 50/100 = 0

    if (error < kI_start_at_error_value)
        {
            integral += error;
        }

    derivative = prev_error - error;

    prev_error = error;

    output = forward_kP*(error + forward_kI*integral + forward_kD*derivative);

    if(output > 11000) //if output is greater than 11.000 volts (out of a possible 12), cap at 11000 milivolts
	{
		output = 11000;
	} 

	//output to drivetrain

	left_drivetrain.move_voltage(output);	//output needs to be between -12000 to 12000 milivolts
	right_drivetrain.move_voltage(output);


	if(abs(error) < settle_distance) //absolute value so it never goes negative
	{
		settle_timer += 20;
	} else
	{
		settle_timer = 0;
	}

    timer += 20;


    loopRate.delay(okapi::QFrequency(50.0)); // Delay to maintain the specified loop rate of 50 cycles per second (20 ms between start of cycles)
}
}


void TurnPID(float target, float settle_time_msec, float kI_start_at_error_value, int timeout_msec)
    {
    float error = target - inertial_sensor.get_rotation(); //target is degrees we want to be at
    float prev_error;
    float integral;
    float derivative;
	float sensor; 
	int output;

	float settle_distance = 1; //change this value to change what error the PID considers "settled"

	int timer = 0;
	int settle_timer = 0;

	if (timeout_msec = -1) //this sets the default timeout_msec based on the error, which we couldn't calculate in the parameters field, so we do it here instead
	{
		timeout_msec = error * 30 + 500; //sets the timeout msecs to 30 times the error plus a baseline 500 ms
	}


while(timer < timeout_msec && settle_timer < settle_time_msec)
    {
	sensor = inertial_sensor.get_rotation();

	error = target - sensor;

    if (error < kI_start_at_error_value)
        {
            integral += error;
        }

    derivative = prev_error - error;

    prev_error = error;

    output = turn_kP*(error + turn_kI*integral + turn_kD*derivative);

    if(output > 11000) //if output is greater than 11.000 volts (out of a possible 12), cap at 11000 milivolts
	{
		output = 11000;
	} 

	//output to drivetrain

	left_drivetrain.move_voltage(output);	//output needs to be -12000 to 12000 milivolts
	right_drivetrain.move_voltage(-output);


	if(abs(error) < settle_distance) //absolute value so it never goes negative
	{
		settle_timer += 20;
	} else
	{
		settle_timer = 0;
	}

    timer += 20;


	loopRate.delay(okapi::QFrequency(50.0)); // Delay to maintain the specified loop rate of 50 cycles per second

    }
}


void flywheel_bang_bang () //BANG BANG control
{ 
while(true) {
	if(toggle_a) {
		int flywheel_rpm;// = flywheel_rotation
		
		if (flywheel_rpm < 3900) {
			flywheel_motor.move_voltage(11500);
		}
		else {
			flywheel_motor.move_voltage(0);
		}
	}
}
}
