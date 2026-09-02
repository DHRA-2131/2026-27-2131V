#include "main.h"
#include "odometry.hpp"
#include "screen.hpp"
#include "autonomous.hpp"
#include "drivetrain.hpp"
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "constants.h"
#include "definitions.h"
#include <cmath>

void initialize() {
	imu.reset();
	while(imu.is_calibrating()){
		pros::screen::print(pros::E_TEXT_LARGE, 0, "Initializing...");
		pros::delay(20);
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	in_autonomous = true;
	if(autonomous_mode == 1){
		//Right autonomous

	}
	else if(autonomous_mode == -1){
		//Left autonomous

	}
	else if(autonomous_mode == 0){
		//No autonomous (move forward 1)
		drivetrain::move_forward(1, 127);
	}
	else if(autonomous_mode == 2){
		//Autonomous skills

	}
	
}

void opcontrol(){
	//pros::Task storage_task(move_storage, nullptr);
	//pros::Task intake_task(move_intake, nullptr);
	in_autonomous = false;
	DrivetrainL.set_gearing(pros::E_MOTOR_GEARSET_36);
	DrivetrainR.set_gearing(pros::E_MOTOR_GEARSET_36);
	DrivetrainL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	DrivetrainR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	redrawscreen();
	while(true){
		if(driving_mode == 0){ //Tank control
			DrivetrainL.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127);
			DrivetrainR.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * driving_speed / 127);
		}
		else if(driving_mode == 1){ //Arcade control
			double dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127;
			double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * driving_speed / 127;
			DrivetrainL.move(dir - turn);
			DrivetrainR.move(dir + turn);
		}	
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			Lift.move(30);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			Lift.move(-20);
		}
		else{
			Lift.move(0);
		}
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			ColorSwitcher.move(127);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			ColorSwitcher.move(-127);
		}
		else{
			ColorSwitcher.move(0);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1){
			driving_speed = 127;
		} 
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) == 1){
			driving_speed = 50;
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) == 1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 0){
			autonomous();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == 1){
			if(autonomous_mode == 1){
				autonomous_mode = -1;		
			}
			else if(autonomous_mode == -1){
				autonomous_mode = 0;	
			}
			else if(autonomous_mode == 0){
				autonomous_mode = 2;
			}
			else if(autonomous_mode == 2){
				autonomous_mode = 1;
			}
			redrawscreen();
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == 1){
			if(driving_mode == 0){
				driving_mode = 1;
			}
			else if(driving_mode == 1){
				driving_mode = 0;
			}
			redrawscreen();
		}
		pros::delay(20);
	}
}
