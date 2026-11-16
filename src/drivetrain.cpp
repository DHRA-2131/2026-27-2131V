#include "drivetrain.hpp"
#include "definitions.hpp"
#include "constants.hpp"
#include "odometry.hpp"
#include "field.hpp"

PID drivePID(3.0, 0, 0); // PID controller for driving forward/backward
PID turnPID(1, 0, 0);  // PID controller for turning left/right

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

	//Save the positions of the two sides
	double start_left = drivetrainL.get_position();
	double start_right = drivetrainR.get_position();

	//Number of attempts to get the position to 0 while past the threshold.
	int attempt = 0;

	while(true){ 
		//Get the average of the two motor group positions and multiply by the constant to get how far the robot has moved.
		double drive_pos = (drivetrainL.get_position() - start_left + drivetrainR.get_position() - start_right) / 2.0 * DRIVE_DIST_PER_DEG;

		//Calculate the how far away the robot is from the target
		double error = target - drive_pos;

		//Calculate the output using the PID controller
		double output = drivePID.update(error);

		//Move the motors using the calculated output
		drivetrain.move(output, output);

		//Check if the error is small enough
		if(std::abs(error) < 1.0){

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
	//Update odometry
	odometry.update_position();
}

void Drivetrain::move_backward(double target){
	//If moving backward, use the move_forward() function but use a negative value
	drivetrain.move_forward(-target);
}

void Drivetrain::turn_left(double target){
	//This function turns the robot left a certain amount of degrees.
	
	//Reset the PID controller
	turnPID.reset(); 
	
	//Number of attempts to get the position to 0 while past the threshold.
	int attempt = 0;

	//Get the heading before the loop
	double start_heading = imu.get_heading();

	while(true){ 

		//Calculate the scaled degrees turned
		double degrees_turned = (imu.get_heading() - start_heading) * IMU_DRIFT_SCALE_FACTOR;

		//Wrap to [-180,180]
		while (degrees_turned > 180.0) { degrees_turned -= 360.0; }
    	while (degrees_turned < -180.0) { degrees_turned += 360.0; }

		//Calculate the how far away the robot is from the target
		double error = target - degrees_turned;

		//Wrap the error 
		while(error > 180.0) {error -= 360.0;}
		while(error < -180.0) {error += 360.0;}

		//Calculate the output using the PID controller
		double output = turnPID.update(error);

		//Move the motors using the calculated output
		drivetrain.move(-output, output);

		//Check if the error is small enough
		if(std::abs(error) < 1.0){

			//Increment the attempt by 1
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
	//Update odometry
	odometry.update_position();
}

void Drivetrain::turn_right(double target){
	//When turning right, use turn_left() but with a negative value.
	drivetrain.turn_left(-target);
}

void Drivetrain::turn_to_heading(double target){

	//Subtract the current heading from the target to get the angle we need to move
	double angle = target - imu.get_heading() * IMU_DRIFT_SCALE_FACTOR;

	//Wrap the angle to [-180,180]
	while(angle > 180.0) {angle -= 360.0;}
    while(angle < -180.0) {angle += 360.0;}

	//Turn to that angle
	turn_left(angle);
}

void Drivetrain::goto_xy(double target_x, double target_y){
	//This function will take an x and y coordinate, turn toward it, and move to those coordinates.
	
	//Find the distance we need to move on the x and y coordinates
	double dist_x = target_x - pos_x;
	double dist_y = target_y - pos_y;
	
	//Calculate the angle we need to turn to by taking the arctangent of y/x
	double target_angle = std::atan2(dist_y, dist_x);
	
	//Calculate the minimum and maximum x and y positions with a small buffer to ensure that the robot does not hit an obstacle.
	extent_x = std::fabs(std::cos(target_angle)) * (ROBOT_LENGTH_IN / 2.0)
		+ std::fabs(std::sin(target_angle)) * (ROBOT_WIDTH_IN / 2) + 0.5;
	extent_y = std::fabs(std::sin(target_angle)) * (ROBOT_LENGTH_IN / 2.0)
		+ std::fabs(std::cos(target_angle)) * (ROBOT_WIDTH_IN / 2) + 0.5;
	
	//Check if there are obstacles. If not, take a straight line path to get there.
	if(!check_for_obstacles(target_x, target_y)){
		//Turn to that angle
		turn_to_heading(target_angle * RAD_TO_DEG);

		//Calculate the distance we need to move to reach the target by taking the secant of the angle and multiplying by the x distance
		double target_distance = (1.0 / std::cos(target_angle * DEG_TO_RAD)) * dist_x;
		
		//Move that distance
		move_forward(target_distance);
	} else {
		//If there are obstacles, move to a location halfway that avoids the obstacle and then move to the location
		goto_xy(target_x + extent_x + 1, target_y / 2);
		goto_xy(target_x, target_y);

	}
}

bool Drivetrain::check_for_obstacles(double target_x, double target_y){
	//This function will check if an obstacle is in the way of the robot when it is trying to reach specific coordinates.
	
	
	//Calculate the distance moved by the robot
	double distance = hypot(target_x - pos_x, target_y - pos_y);
	if (distance <= 0.0) {
		return false;
	}
	
	//Calculate the amount of steps that the code will take to make sure that the robot will not collide
	int steps = static_cast<int>(std::ceil(distance));
	if (steps <= 0) {
		return false;
	}
	
	for(int step = 0; step <= steps; step++){
		//Progress until robot is done checking
		double progress = static_cast<double>(step) / steps;
		
		//Current x and y position that we are checking
		double x = pos_x + (target_x - pos_x) * progress;
		double y = pos_y + (target_y - pos_y) * progress;
		
		//Check if the robot's path will collide with an obstacle
		for(int i = 0; i < 4; i++){
			if(x >= Field::obstacles[i][0][0] - extent_x
				&& x <= Field::obstacles[i][0][1] + extent_x
				&& y >= Field::obstacles[i][1][0] - extent_y
				&& y <= Field::obstacles[i][1][1] + extent_y){
					return true;
				}
			}
	}
	return false;
}