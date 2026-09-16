#include "main.h"
#include "definitions.hpp"
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "constants.hpp"
#include <cmath>
#include <string>

void initialize() {
	imu.reset();
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
		drivetrain.move(1, 127);
	}
	else if(autonomous_mode == 2){
		//Autonomous skills

	}
	
}

void opcontrol(){
	//pros::Task storage_task(move_storage, nullptr);
	//pros::Task intake_task(move_intake, nullptr);
	in_autonomous = false;
	drivetrainL.set_gearing(pros::E_MOTOR_GEARSET_36);
	drivetrainR.set_gearing(pros::E_MOTOR_GEARSET_36);
	drivetrainL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drivetrainR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	screen.redrawscreen();
	while(imu.is_calibrating()){
		screen.print("Initializing...");
		pros::delay(20);
	}
	while(true){
		if(driving_mode == 0){ //Tank control
			drivetrainL.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127);
			drivetrainR.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * driving_speed / 127);
		}
		else if(driving_mode == 1){ //Arcade control
			double dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127;
			double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * driving_speed / 127;
			drivetrainL.move(dir - turn);
			drivetrainR.move(dir + turn);
		}	
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			lift.move(30);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			lift.move(-20);
		}
		else{
			lift.move(0);
		}
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			colorSwitcher.move(127);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			colorSwitcher.move(-127);
		}
		else{
			colorSwitcher.move(0);
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
			screen.redrawscreen();
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) == 1){
			if(driving_mode == 0){
				driving_mode = 1;
			}
			else if(driving_mode == 1){
				driving_mode = 0;
			}
			screen.redrawscreen();
		}
		pros::delay(20);
	}
}
