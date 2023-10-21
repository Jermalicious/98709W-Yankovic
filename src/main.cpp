#include "main.h"
#include "classesTesting.h"
#include "autonFunctions.h"

//define classes from classesTesting.h
	CustomMath customMath;

// define controller
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

//define miscellaneous motors, pneumatics, and tracking wheels 
	pros::Motor left_intake(7,false);			//the left intake motor
	pros::Motor right_intake(5,true); 		//the right intake motor
	pros::ADIDigitalOut wings (1, LOW); 	//the pneumatics to extend the pusher wings
	pros::Rotation tracking_wheel_X (13, false); //Change Port
	pros::Rotation tracking_wheel_Y	(12, false); //Change Port
	pros::Imu inertial_sensor (18);

//define drivetrain motors
	pros::Motor left_top_drive (2,true);
	pros::Motor left_back_drive (3,false);
	pros::Motor left_front_drive (4, true);
	pros::Motor right_top_drive (10,false);
	pros::Motor	right_back_drive (9,true);
	pros::Motor right_front_drive (8,false);

//define drivetrain motor groups
	pros::Motor_Group left_drivetrain({left_top_drive, left_back_drive, left_front_drive}); 	//the three motors for the left side of the drivetrain
	pros::Motor_Group right_drivetrain({right_top_drive, right_back_drive, right_front_drive}); //the three motors for the right side of the drivetrain
	pros::Motor_Group intake({left_intake, right_intake});


void drive(std::array <float,2> drive_input)
{
	left_drivetrain.move_voltage(drive_input[1]);
	right_drivetrain.move_voltage(drive_input[2]);
}

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
		PIDForward forward(.8, .01, .2);
		PIDTurn turn(.8, .01, .2);
		inertial_sensor.reset();

		drive(forward.run(tracking_wheel_Y, 48)); //drive to middle

		drive(turn.run(inertial_sensor.get_rotation(), 90)); //turn to goal

		drive(forward.run(tracking_wheel_Y, 6)); //drive to goal

		intake = -95; //deopist tri-ball
		pros::delay(500);
		intake.brake();


		drive(forward.run(tracking_wheel_Y, -6)); //back up from goal

		drive(turn.run(inertial_sensor.get_rotation(), 53.1)); //turn toward elevation bar

		wings.set_value(HIGH); //extend wings to catch on bar

		drive(forward.run(tracking_wheel_Y, 60)); //drive to the bar
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

 
//test printing stuff
	// pros::lcd::print(3,"%s",test.get_bob());

//decalre variables

float drive_forward;
float drive_turn;

float left_drive_speed;
float right_drive_speed;

float const drive_turn_constant = 1.4;

//initialize variables


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Control Loop///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	while (true){

//printing on the brain screen
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);


	//intake controller
		if (controller.get_digital(DIGITAL_R1)) { //forward

			intake = 95;

		} else if (controller.get_digital(DIGITAL_L1)) { //reverse

			intake = -95;

		} else {

			intake.brake();

		}

	//wings controller
		if (controller.get_digital(DIGITAL_X))
		{ 
			wings.set_value(HIGH);
		} else
		{
			wings.set_value(LOW);
		}

	//Sets drivetrain speed in % (capped at 95%)
		drive_forward = controller.get_analog(ANALOG_LEFT_Y);
		drive_turn = controller.get_analog(ANALOG_RIGHT_X);

		left_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant + drive_turn);
		right_drive_speed = customMath.drive_cubic(drive_forward * drive_turn_constant - drive_turn);

		left_drivetrain = left_drive_speed;
		right_drivetrain = right_drive_speed;


		pros::delay(20);
	
	}
}
