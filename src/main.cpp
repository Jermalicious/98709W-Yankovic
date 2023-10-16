#include "main.h"
#include "classesTesting.h"
#include "autonFunctions.h"

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

//define classes from classesTesting.h
	Testing test;
	CustomMath customMath;

// define controller
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

//define miscellaneous motors and pneumatics
	pros::Motor left_armo(7,false);			//the left intake motor
	pros::Motor right_armo(5,true); 		//the right intake motor
	pros::ADIDigitalOut wings (1, LOW); 	//the pneumatics to extend the pusher wings

//define drivetrain motors
	pros::Motor left_top_drive (2,true);
	pros::Motor left_back_drive (3,false);
	pros::Motor left_front_drive (4, true);				//left_front_motor
	pros::Motor right_top_drive (10,false);
	pros::Motor	right_back_drive (9,true);
	pros::Motor right_front_drive (8,false);

//define drivetrain motor groups
	pros::Motor_Group left_drivetrain({left_top_drive, left_back_drive, left_front_drive}); 	//the three motors for the left side of the drivetrain
	pros::Motor_Group right_drivetrain({right_top_drive, right_back_drive, right_front_drive}); //the three motors for the right side of the drivetrain
 
//test printing stuff
	pros::lcd::print(3,"%s",test.get_bob());


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Control Loop///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	while (true){

//printing on the brain screen
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);


		
		if (controller.get_digital(DIGITAL_A)) { //Press A to activate intake forward

			left_armo = 95;
			right_armo = 95;
		} else if (controller.get_digital(DIGITAL_B)) {

			left_armo = -95;
			right_armo = -95;
		}

		if (controller.get_digital(DIGITAL_X))
		{ 
			wings.set_value(HIGH);
		} else
		{
			wings.set_value(LOW);
		}

		float left_drive_speed = customMath.drive_cubic(controller.get_analog(ANALOG_LEFT_Y));
		float right_drive_speed = customMath.drive_cubic(controller.get_analog(ANALOG_RIGHT_Y));

		left_drivetrain = left_drive_speed;
		right_drivetrain = right_drive_speed;


		pros::delay(20);
	
	}
}
