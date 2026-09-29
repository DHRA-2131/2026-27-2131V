#include "drivetrain.hpp"
#include "main.h"
#include "definitions.hpp"
#include "constants.hpp"
#include "odometry.hpp"
#include <cmath>

PID drivePID(3.0, 0, 0); // PID controller for driving forward/backward
PID turnPID(1.0, 0, 0);  // PID controller for turning left/right
void Drivetrain::move(int powerL, int powerR){
    drivetrainL.move(powerL);
    drivetrainR.move(powerR);
}
void Drivetrain::stop(){
    drivetrainL.move(0);
    drivetrainR.move(0);
}

void Drivetrain::move_forward(double target){
	//This function moved the robot forward a certain amount of inches

	//Reset the PID controller
	drivePID.reset(); 

	//Set the motor positions to 0
	drivetrainL.tare_position();
	drivetrainR.tare_position();

	//Number of attempts to get the position to 0 while past the threshold.
	int attempt = 0;

	while(true){ 
		//Get the average of the two motor group positions and multiply by the constant to get how far the robot has moved.
		double drive_pos = (drivetrainL.get_position() + drivetrainR.get_position() / 2) * DRIVE_DIST_PER_DEG;

		//Calculate the how far away the robot is from the target
		double error = target - drive_pos;

		//Calculate the output using the PID controller
		double output = drivePID.update(error);

		//Move the motors using the calculated output
		drivetrain.move(output, output);

		//Check if the error is small enough
		if(std::abs(error) < 1){

			//Increment the attempt by 1;
			attempt++;

			if(attempt > 3){
				//If the error is small enough stop the drivetrain and break out of the loop.
				drivetrain.stop();
				break;
			} 
		} else {
			//If the error is not small enough, reset the attempt counter to 0.
			attempt = 0;
		}
		//Delay to not hog the CPU
		pros::delay(20);
	}
}

void Drivetrain::move_backward(double target){
	//If moving backward, use the move_forward() function but use a negative value
	drivetrain.move_forward(-target);
}

void Drivetrain::turn_left(double target){
	//This function turns the robot left a certain amount of degrees.
	
	//Reset the PID controller
	turnPID.reset(); 

	//Wrap the target angle to [-180,180]
	while (target > 180) {target -= 360;}
    while (target < -180) {target += 360;}

	//Number of attempts to get the position to 0 while past the threshold.
	int attempt = 0;

	while(true){ 
		//Multiply the heading by the scale factor to account for scaling drift on the imu.
		double heading = imu.get_heading() * IMU_DRIFT_SCALE_FACTOR;

		//Calculate the how far away the robot is from the target
		double error = target - heading;

		//Calculate the output using the PID controller
		double output = turnPID.update(error);

		//Move the motors using the calculated output
		drivetrain.move(-output, output);

		//Check if the error is small enough
		if(std::abs(error) < 1){

			//Increment the attempt by 1;
			attempt++;

			if(attempt > 3){
				//If the error is small enough stop the drivetrain and break out of the loop.
				drivetrain.stop();
				break;
			} 
		} else {
			//If the error is not small enough, reset the attempt counter to 0.
			attempt = 0;
		}
		//Delay to not hog the CPU
		pros::delay(20);
	}
}

void Drivetrain::turn_right(double target){
	//When turning right, use turn_left() but with a negative value.
	drivetrain.turn_left(-target);
}

void Drivetrain::turn_to_heading(double target){
	//Multiply the target by the IMU drift scale factor
	target *= IMU_DRIFT_SCALE_FACTOR;

	//Subtract the current heading from the target to get the angle we need to move
	double angle = target - imu.get_heading();

	//Wrap the angle to [-180,180]
	while (angle > 180) {angle -= 360;}
    while (angle < -180) {angle += 360;}

	//Turn to that angle
	turn_left(angle);
}

void Drivetrain::goto_xy(double target_x, double target_y){
	//This function will take an x and y coordinate, turn toward it, and move to those coordinates.

	//Find the distance we need to move on the x and y coordinates
	double dist_x = target_x - odometry.pos_x;
	double dist_y = target_y - odometry.pos_y;

	//Calculate the angle we need to turn to by taking the arctangent of y/x
	double target_angle = atan(dist_y / dist_x);

	//Turn to that angle
	turn_to_heading(target_angle);
	
	//Calculate the distance we need to move to reach the target by taking the secant (1/cos) of the angle and multiplying by the x distance
	double target_distance = (1 / cos(target_angle)) * dist_x;

	//Move that distance
	move_forward(target_distance);
}