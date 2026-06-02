#include "pros/apix.h"
#include "pros/imu.hpp"

bool in_autonomous;
bool arm_toggle = false;
bool match_loader_toggle = false;
float driving_speed = 127;
//int storage_control = 0;
//int intake_control = 0;
int autonomous_mode = 1;
int driving_mode = 1;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup DrivetrainL({-19, -20}); 
pros::MotorGroup DrivetrainR({11, 12});    
pros::Motor DrivetrainLF(11);
pros::IMU imu(15);
double old_heading;
double new_heading;
double old_rotation;
double new_rotation;
double pos_x;
double pos_y;
