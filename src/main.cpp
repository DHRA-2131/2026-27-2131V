#include "definitions.hpp"
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "constants.hpp"
#include <string>

void initialize() {
	imu.reset();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	in_autonomous = true;
	if(autonomous_mode == 1){
		drivetrain.turn_left(1080);
		
	}
	else if(autonomous_mode == -1){
		//Left autonomous
		drivetrain.move_forward(24);
		
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
	//Configure motors
	drivetrainL.set_gearing(pros::E_MOTOR_GEARSET_36);
	drivetrainR.set_gearing(pros::E_MOTOR_GEARSET_36);
	drivetrainL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drivetrainR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	clawSpinner.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	screen.redraw();

	while(imu.is_calibrating()){
		screen.print("Initializing...");
		pros::delay(20);
	}
	while(true){
		if(driving_mode == 0){ //Tank control
			drivetrain.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127, master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * driving_speed / 127);
		}
		else if(driving_mode == 1){ //Arcade control
			double dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * driving_speed / 127;
			double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * driving_speed / 127;
			drivetrain.move(dir - turn, dir + turn);
		}	

		//If either joystick is at 0 (with a small buffer), update the odometry.
		if(fabs(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0.5 || fabs(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0.5){
			if(!updated){
				odometry.update_position();
				updated = true;
			}
		} else {
			updated = false;
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			lift.move(40);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			lift.move(-30);
		}
		else{
			lift.move(0);
		}
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			claw.move(64);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			claw.move(-64);
		}
		else{
			claw.move(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			clawSpinner.move(64);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			clawSpinner.move(-64);
		}
		else{
			clawSpinner.move(0);
		}


		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			driving_speed += 20;
			if(driving_speed > 127){
				driving_speed = 127;
			}
		} 
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			driving_speed -= 20;
			if(driving_speed < 0){
				driving_speed = 0;
			}
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			autonomous();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
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
			screen.redraw();
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			if(driving_mode == 0){
				driving_mode = 1;
			}
			else if(driving_mode == 1){
				driving_mode = 0;
			}
			screen.redraw();
		}
		pros::delay(20);
	}
}
