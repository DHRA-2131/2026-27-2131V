#pragma once
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "screen.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"
#include "pid.hpp"
extern bool in_autonomous;
extern float driving_speed;
extern int autonomous_mode;
extern int driving_mode;
extern pros::Controller master;
extern pros::MotorGroup drivetrainL; 
extern pros::MotorGroup drivetrainR;    
extern pros::MotorGroup lift;
extern pros::Motor colorSwitcher;
extern pros::Motor trackingMotor;
extern pros::IMU imu;
extern Screen screen;
extern Drivetrain drivetrain;
extern Odometry odometry;
extern PID pid;