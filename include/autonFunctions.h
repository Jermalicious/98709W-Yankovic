// #include <fstream>
// #include <iostream>


//define miscellaneous motors, pneumatics, and tracking wheels 
	pros::Motor left_intake(7,true);		//the left intake motor
	pros::Motor right_intake(??,false); 	//(not plugged in) the right intake motor
    pros::Motor cata_motor (5,false);       //the catapult motor
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



class Controll
    {
    
    public:
        Controll();

    protected:

        float basicPID(double kP, double kI, double kD, float sensor_input, float target, float kI_start_at_error_value, float timeout_msec);

        double kP;
        double kI;
        double kD;

    private:
        int timer;

    };

class PIDForward : public Controll
    {
    public:

        PIDForward(double input_kP, double input_kI, double input_kD) //PIDForward constructor. Sets kP, kI, and kD values from an input. Later input from a file?
        {
            kP = input_kP;
            kI = input_kI;
            kD = input_kD;
        }

        void run(float target_inches, float kI_start_at_error_value = 10, float timeout_msec = 3000) //Add settle time. This returns an array with two items: left_speed and right_speed
        {
            float sensor_input = tracking_wheel_Y.get_angle() / 360 * 10.2 /* tracking wheel circumference*/;

            float forward_speed = basicPID(kP, kI, kD, sensor_input, target_inches, kI_start_at_error_value, timeout_msec);  

            left_drivetrain, right_drivetrain = forward_speed; //sets both left and right drivetrain sides to the output of the PID

        }

    };


class PIDTurn : public Controll
    {
    public:

        PIDTurn(double input_kP, double input_kI, double input_kD) //PIDTurn constructor. Sets kP, kI, and kD values from an input. Later input from a file?
        {
            kP = input_kP;
            kI = input_kI;
            kD = input_kD;
        }

        void run(float target_inches, float kI_start_at_error_value = 10, float timeout_msec = 3000) //Add settle time. This returns an array with two items: left_speed and right_speed
        {
            float sensor_input = inertial_sensor.get_rotation(); /* tracking wheel circumference*/;

            float turn_speed = basicPID(kP, kI, kD, sensor_input, target_inches, kI_start_at_error_value, timeout_msec);  

            left_drivetrain, right_drivetrain = turn_speed, -turn_speed; //sets left and right to the same speed, opposite directions as the PID outputs

        }

    };


class Odometry
    {
    public:

        void getPostition();
        void moveTo(float x, float y, float orientation);
        void lineFollow();

    protected:

    };

float Controll::basicPID(double kP, double kI, double kD, float sensor_input, float target, float kI_start_at_error_value, float timeout_msec)
    {
    float error = target - sensor_input;
    float prev_error;
    float integral;
    float derivative;

    timer = 0;

    while(timer <= timeout_msec && timer < settle_time_msec)
    {

    if (error < kI_start_at_error_value)
        {
            integral += error;
        }

    derivative = prev_error - error;

    prev_error = error;

    float output = kP*error + kI*integral + kD*derivative;

    if(output > 11) //if output is greater than 11 volts (out of a possible 12), cap at 11 volts
    {
        output = 11;
    }

    timer += 20;
    return output;
    }
}

Controll::Controll()
{

}