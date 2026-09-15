#pragma once
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "screen.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
extern bool in_autonomous;
extern float driving_speed;
extern int autonomous_mode;
extern int driving_mode;
extern pros::Controller master;
extern pros::MotorGroup DrivetrainL; 
extern pros::MotorGroup DrivetrainR;    
extern pros::MotorGroup Lift;
extern pros::Motor ColorSwitcher;
extern pros::Motor TrackingMotor;
extern pros::IMU imu;
extern Screen screen;
extern Drivetrain drivetrain;
extern Odometry odometry;