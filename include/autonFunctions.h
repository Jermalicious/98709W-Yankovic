// #include <fstream>
// #include <iostream>

class Controll
    {
    
    public:
        Controll();

    protected:

        float basicPID(double kP, double kI, double kD, float sensor_input, float target, float kI_start_at_error_value, float timeout_msec);

        double kP;
        double kI;
        double kD;

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

        std::array< float,2 > &run(pros::Rotation sensor, float target_inches, float kI_start_at_error_value = 10, float timeout_msec = 3000) //Add settle time. This returns an array with two items: left_speed and right_speed
        {
            float sensor_input = sensor.get_position() / 360 * 10.2 /* tracking wheel circumference*/;

            float forward_speed = basicPID(kP, kI, kD, sensor_input, target_inches, kI_start_at_error_value, timeout_msec);  
            std::array < float,2 > left_right_speed = {forward_speed,forward_speed}; //std::array < float,2 > is a data type. it is an array with two floats in it

            return left_right_speed;
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

        std::array< float,2 > &run(float sensor_input, float target_degrees, float kI_start_at_error_value = 45, float timeout_msec = 2.5) //Add settle time. This returns an array with two items: left_speed and right_speed
        {


            float turn_speed = basicPID(kP, kI, kD, sensor_input, target_degrees, kI_start_at_error_value, timeout_msec);  
            std::array < float,2 > left_right_speed = {turn_speed,-turn_speed};

            return left_right_speed;
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

float basicPID (double kP, double kI, double kD, float sensor_input, float target, float kI_start_at_error_value, float timeout_msec)
    {
    float error = target - sensor_input;
    float prev_error;
    float integral;
    float derivative;

    if (error < kI_start_at_error_value)
        {
            integral += error;
        }

    derivative = prev_error + error;

    prev_error = error;

    float output = kP*error + kI*integral + kD*derivative;

    if(output > 11) //if output is greater than 11 volts (out of a possible 12), cap at 11 volts
    {
        output = 11;
    }

    return output;
    }

Controll::Controll()
{

}