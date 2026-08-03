#include "main.h"
#include "definitions.h"
#include "constants.h"
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
	pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Position: (" + std::round(pos_x) + ", " + std::round(pos_y) + ")");
}
