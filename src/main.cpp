#include "main.h"
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "constants.h"
#include "definitions.h"
void move_forward(float dist, int power){
	float target = DrivetrainL.get_position() + (dist / (M_PI * WHEEL_DIAMETER_IN)) * 360.0;
	while(DrivetrainL.get_position() <= target){
		DrivetrainL.move(power);
		DrivetrainR.move(power);
		pros::delay(20);
	}
	DrivetrainL.move(0);
	DrivetrainR.move(0);
}

void move_backward(float dist, int power){
	float target = DrivetrainL.get_position() - (dist / (M_PI * WHEEL_DIAMETER_IN)) * 360.0;
	while(DrivetrainL.get_position() >= target){
		DrivetrainL.move(-power);
		DrivetrainR.move(-power);
		pros::delay(20);
	}
	DrivetrainL.move(0);
	DrivetrainR.move(0);
}

void turn_left(float angle, int power){
	float old_rotation = imu.get_rotation() - angle;
	while(imu.get_rotation() >= old_rotation){
		DrivetrainL.move(-power);
		DrivetrainR.move(power);
		pros::delay(20);
	}
	DrivetrainL.move(0);
	DrivetrainR.move(0);
}

void turn_right(float angle, int power){
	float old_rotation = imu.get_rotation() + angle;
	while(imu.get_rotation() <= old_rotation){
		DrivetrainL.move(power);
		DrivetrainR.move(-power);
		pros::delay(20);
	}
	DrivetrainL.move(0);
	DrivetrainR.move(0);
}
/*
void move_intake(void*) {
	while (true) {
		if (in_autonomous) {
			if (intake_control == 1) {
				Intake.move(127);
			}
			else if (intake_control == -1) {
				Intake.move(-127);
			}
			else { 
				Intake.move(0);
			}
		}
		pros::delay(20);
	}
}

void move_storage(void*) {
	while (true) {
		if (in_autonomous) {
			if (storage_control == 1) {
				Storage.move(127);
			}
			else if (storage_control == -1) {
				Storage.move(-127);
			}
			else {
				Storage.move(0);
			}
		}
		pros::delay(20);
	}
}

*/
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
		move_forward(1, 127);
	}
	else if(autonomous_mode == 2){
		//Autonomous skills

	}
	
}
void redrawscreen(){
	pros::screen::erase();
	if(driving_mode == 1){
		pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Driving mode: arcade");
	}
	else if(driving_mode == 0){
		pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Driving mode: tank");
	}
	if(autonomous_mode == 0){
		pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Autonomous mode: none");
	}
	else if(autonomous_mode == 1){
		pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Autonomous mode: right");
	}
	else if(autonomous_mode == -1){
		pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Autonomous mode: left");
	}
	else if(autonomous_mode == 2){
		pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Autonomous mode: skills");
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
	redrawscreen();
	while(true){
		if(driving_mode == 0){
			DrivetrainL.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127);
			DrivetrainR.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * driving_speed / 127);
		}
		else if(driving_mode == 1){
			double dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127;
			double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * driving_speed / 127;
			DrivetrainL.move(dir + turn);
			DrivetrainR.move(dir - turn);
		}	
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1){
			driving_speed = 127;
		} 
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 1){
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
