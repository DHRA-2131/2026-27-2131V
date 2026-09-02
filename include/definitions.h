#pragma once
#include "pros/apix.h"
#include "pros/imu.hpp"
bool in_autonomous;
float driving_speed = 127;
int autonomous_mode = 1;
int driving_mode = 1;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup DrivetrainL({-19, -20}); 
pros::MotorGroup DrivetrainR({11, 12});    
pros::MotorGroup Lift({7, -8, -9, 10});
pros::Motor ColorSwitcher(5);
pros::Motor TrackingMotor(-19);
pros::IMU imu(5);