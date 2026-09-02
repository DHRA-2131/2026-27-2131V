#include "main.h"
#include "definitions.h"
#include "constants.h"
#include <string>
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
	//string text = "Position: (" + std::to_string(std::round(pos_x)) + ", " + std::to_string(std::round(pos_y)) + ")"
	//pros::screen::print(pros::E_TEXT_MEDIUM, 2, text);
}
