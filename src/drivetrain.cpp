#include "drivetrain.hpp"
#include "main.h"
#include "definitions.hpp"
#include "constants.hpp"
#include "odometry.hpp"
void Drivetrain::move(int powerL, int powerR){
    DrivetrainL.move(powerL);
    DrivetrainR.move(powerR);
}
void Drivetrain::stop(){
    DrivetrainL.move(0);
    DrivetrainR.move(0);
}

void Drivetrain::move_forward(double dist, int power){
	target = DrivetrainL.get_position() + (dist * GEAR_RATIO / (M_PI * WHEEL_DIAMETER_IN)) * 360.0;
	while(DrivetrainL.get_position() <= target){
		drivetrain.move(power, power);
		pros::delay(20);
	}
	drivetrain.stop();
}

void Drivetrain::move_backward(double dist, int power){
	target = DrivetrainL.get_position() - (dist * GEAR_RATIO / (M_PI * WHEEL_DIAMETER_IN)) * 360.0;
	while(DrivetrainL.get_position() >= target){
		drivetrain.move(-power, -power);
		pros::delay(20);
	}
	drivetrain.stop();
}

void Drivetrain::turn_left(double angle, int power){
	target = imu.get_rotation() - angle;
	while(imu.get_rotation() >= target){
		drivetrain.move(-power, power);
		pros::delay(20);
	}
	drivetrain.stop();
}

void Drivetrain::turn_right(double angle, int power){
	target = imu.get_rotation() + angle;
	while(imu.get_rotation() <= target){
		drivetrain.move(power, -power);
		pros::delay(20);
	}
	drivetrain.stop();
}
