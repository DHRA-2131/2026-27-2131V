#include "odometry.hpp"
#include "constants.hpp"
#include "definitions.hpp"
#include <cmath>

Odometry::Odometry() : change_x(0.0), change_y(0.0), old_heading(0.0), old_rotation(0.0), new_heading(0.0), new_rotation(0.0) {
    old_heading = imu.get_heading();
    old_rotation = imu.get_rotation();
}

void Odometry::update_position(){
    //This is an arc odometry system. To fully understand how it works, visit https://dhra-2131.github.io/website/programming/control-theory-for-vex/localization/odometry/

    //Get where robot is facing
    new_heading = imu.get_heading();
    new_rotation = imu.get_rotation();

    //See how far the robot has moved by averaging the position of the left and right drivetrain and multiplying by the constant. This is represented as an arc.
    double distance_moved = ((drivetrainL.get_position() + drivetrainR.get_position()) / 2.0) * DRIVE_DIST_PER_DEG;

    //Convert these to radians for use in the trig functions.
    //Calculate how much the heading has changed since the start of the move (this will be a small number)
    double change_in_heading = new_heading - old_heading * DEG_TO_RAD;
    while (change_in_heading > M_PI) {change_in_heading -= (2.0 * M_PI);}
    while (change_in_heading < -M_PI) {change_in_heading += (2.0 * M_PI);}

    //Calculate the average of the start and end headings.
    double average_heading = ((new_heading + old_heading) / 2.0) * DEG_TO_RAD;

    //If the robot is moving straight, use a linear formula instead of the arc formula to prevent dividing by zero.
    if (std::abs(change_in_heading) < 1e-9) {
        change_x = distance_moved * std::cos(average_heading);
        change_y = distance_moved * std::sin(average_heading);
    } else {
        //Calculate the radius of the circle that the arc lies on
        double circle_radius = distance_moved / change_in_heading;

        //Calculate the length of a chord between the two endpoints of the arc
        double chord_length = circle_radius * 2.0 * std::sin(change_in_heading / 2.0);

        //Calculate vectors representing the change in x and y
        change_x = chord_length * std::cos(average_heading);
        change_y = chord_length * std::sin(average_heading);
    }

    //Add the change to the x and y position
    pos_x += change_x;
    pos_y += change_y;

    //Reset the values
    old_rotation = new_rotation;
    old_heading = new_heading;
    change_x = 0.0;
    change_y = 0.0;
}
