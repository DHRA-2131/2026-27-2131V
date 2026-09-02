#pragma once
#include "definitions.h"
#include "constants.h"
class odometry{
    private:
        double change_x;
        double change_y;
        double old_heading;
        double old_rotation;
        double new_heading;
        double new_rotation;
    public:
        double pos_x;
        double pos_y;
        void update_position();
};