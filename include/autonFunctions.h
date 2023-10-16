// #include <fstream>
// #include <iostream>

class Controll
    {
    
    public:
        Controll();

    protected:

        float basicPID(double kP, double kI, double kD, float sensor_input, float target, float kI_start_value, float timeout_msec);

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

        float run(float sensor_input, float target, float kI_start_value, float timeout_msec) //later make this constructor search a file on a microSD card to find kP, kI, and kD
        {
            
            return basicPID(kP, kI, kD, sensor_input, target, kI_start_value, timeout_msec);
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

        float run(float sensor_input, float target, float kI_start_value, float timeout_msec) //Add settle_time
        {

            return basicPID(kP, kI, kD, sensor_input, target, kI_start_value, timeout_msec);  
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

float basicPID (double kP, double kI, double kD, float sensor_input, float target, float kI_start_value, float timeout_msec)
    {
    float error = target - sensor_input;
    float prev_error;
    float integral;
    float derivative;

    if (error < kI_start_value)
        {
            integral += error;
        }

    derivative = prev_error + error;

    prev_error = error;

    float output = kP*error + kI*integral + kD*derivative;

    return output;
    }

Controll::Controll()
{

}