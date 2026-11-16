#pragma once
class Odometry{
    private:
        double change_x;
        double change_y;
        double old_heading;
        double old_rotation;
        double new_heading;
        double new_rotation;
        double old_left;
        double old_right;
    public:
        Odometry();
        void update_position();
};