#include "pros/apix.h"
#include "pros/imu.hpp"
#include "screen.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
bool in_autonomous;
float driving_speed = 127;
int autonomous_mode = 1;
int driving_mode = 1;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup drivetrainL({-19, -20}); 
pros::MotorGroup drivetrainR({11, 12});    
pros::MotorGroup lift({7, -8, -9, 10});
pros::Motor colorSwitcher(5);
pros::Motor trackingMotor(-19);
pros::IMU imu(5);
Screen screen;
Drivetrain drivetrain;
Odometry odometry;