#pragma once
class PID{
    private:
        double kP; //Proportional coefficient
        double kI; //Integral coefficient
        double kD; //Derivative coefficient
        double integral_start; //Minimum value 
        double total_error;
        double previous_error;
        double error;
    public:
        PID(double p, double i, double d, double integral_start = 5.0);
        double update(double error);
        double reset();
};