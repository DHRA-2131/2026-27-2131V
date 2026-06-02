#include "constants.h"
#include "definitions.h"
void update_position(){
    double average_heading = new_heading - old_heading;
    double distance = (new_rotation - old_rotation) * DEG_TO_RAD * WHEEL_DIAMETER_IN / 2;
    double change_x = distance * cos(average_heading);
    double change_y = distance * sin(average_heading);
    pos_x += change_x;
    pos_y += change_y;
}