#include "pid.hpp"
#include <cmath>
PID::PID(double kP, double kI, double kD, double integral_start)
: kP(kP), kI(kI), kD(kD), integral_start(integral_start) {
    reset();
}
double PID::reset() {
    total_error = 0;
    previous_error = 0;
    double derivative = error - previous_error;
    if (std::abs(error) > integral_start) {
        total_error += error;
    } else {
        total_error = 0;
    }
    total_error *= .9;
    previous_error = error;
    return (kP * error + kI * total_error + kD * derivative);
}