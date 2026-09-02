#include "drivetrain.hpp"
#include "main.h"
#include "definitions.h"
#include "constants.h"
#include "odometry.hpp"
void drivetrain::move(int powerL, int powerR){
    DrivetrainL.move(powerL);
    DrivetrainR.move(powerR);
}
void drivetrain::stop(){
    DrivetrainL.move(0);
    DrivetrainR.move(0);
}

void drivetrain::move_forward(double dist, int power){
	target = DrivetrainL.get_position() + (dist / (M_PI * WHEEL_DIAMETER_IN)) * 360.0;
	while(DrivetrainL.get_position() <= target){
		drivetrain::move(power, power);
		pros::delay(20);
	}
	drivetrain::stop();
}

void drivetrain::move_backward(double dist, int power){
	target = DrivetrainL.get_position() - (dist / (M_PI * WHEEL_DIAMETER_IN)) * 360.0;
	while(DrivetrainL.get_position() >= target){
		drivetrain::move(-power, -power);
		pros::delay(20);
	}
	drivetrain::stop();
}

void drivetrain::turn_left(double angle, int power){
	target = imu.get_rotation() - angle;
	while(imu.get_rotation() >= target){
		drivetrain::move(-power, power);
		pros::delay(20);
	}
	drivetrain::stop();
}

void drivetrain::turn_right(double angle, int power){
	target = imu.get_rotation() + angle;
	while(imu.get_rotation() <= target){
		drivetrain::move(power, -power);
		pros::delay(20);
	}
	drivetrain::stop();
}
