#include "main.h"
#include "definitions.h"
#include "constants.h"
#include "2131V/odometry.hpp"
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