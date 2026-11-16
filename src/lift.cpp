#include "lift.hpp"
#include "pid.hpp"
#include "definitions.hpp"
#include "constants.hpp"

PID liftPID(0.0,0.0,0.0);
void Lift::move(double target){ //THIS FUNCTION IS NOT COMPLETE; FIND RELATIONSHIP BETWEEN DEGREES AND HEIGHT
	//This function moves the lift a certain amount of inches up or down

	//Reset the PID controller
	liftPID.reset(); 

	//Set the motor positions to 0
	lift.tare_position();

	//Number of attempts to get the position to 0 while past the threshold.
	int attempt = 0;

	while(true){ 
		//Get the average of the two motor positions.
		double pos = (lift.get_position(0) + lift.get_position(1)) / 2.0;

		//Calculate the how far away the robot is from the target
		double error = target - pos;

		//Calculate the output using the PID controller
		double output = liftPID.update(error);

		//Move the motors using the calculated output
		lift.move(output);

		//Check if the error is small enough
		if(std::abs(error) < 1.0){

			//Increment the attempt by 1;
			attempt++;

			if(attempt > 3){
				//If the error is small enough stop the lift and break out of the loop.
				lift.move(0);
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
void Lift::move_to_height(double target){
    
    double change_in_height = target - lift.get_position();
    
    lift.move(change_in_height);
}