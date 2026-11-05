#include "main.h"
#include "definitions.hpp"
#include "constants.hpp"
#include "screen.hpp"
#include <string>
void Screen::print(std::string text){
    pros::screen::print(pros::E_TEXT_MEDIUM, line, text.c_str());
	line++;
}
void Screen::redraw(){

	//Set line counter to 1
	line = 1;

	//Clear the screen
	pros::screen::erase();

	//Print whether the robot is in tank or arcade control mode
	if(driving_mode == 1){
		print("Driving mode: arcade");
	}
	else if(driving_mode == 0){
		print("Driving mode: tank");
	}

	//Print the autonomous mode the robot is in
	if(autonomous_mode == 0){
		print("Autonomous mode: none");
	}
	else if(autonomous_mode == 1){
		print("Autonomous mode: right");
	}
	else if(autonomous_mode == -1){
		print("Autonomous mode: left");
	}
	else if(autonomous_mode == 2){
		print("Autonomous mode: skills");
	}
	
	print("Position: (" + std::to_string(pos_x) + ", " + std::to_string(pos_y) + ")");
}
