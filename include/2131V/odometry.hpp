#include "constants.h"
#include "definitions.h"
void update_position(){
    new_heading = imu.get_heading(); //Where robot is facing
    new_rotation = TrackingMotor.get_position(); //How much the tracking motor has rotated
    double change_in_heading = (new_heading - old_heading) * DEG_TO_RAD;
    double average_heading = ((new_heading + old_heading) / 2) * DEG_TO_RAD;
    double arc_length = (change_in_heading) * WHEEL_DIAMETER_IN;
    double circle_radius = arc_length / change_in_heading;
    double chord_length = circle_radius * 2 * sin(change_in_heading / 2);
    double change_x = chord_length * cos(average_heading);
    double change_y = chord_length * sin(average_heading);
    pos_x += change_x;
    pos_y += change_y;
    old_rotation = new_rotation;
    old_heading = new_heading;
    change_x, change_y = 0;
    

}
