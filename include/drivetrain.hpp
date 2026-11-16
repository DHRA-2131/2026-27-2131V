#pragma once
class Drivetrain{
    private:
        double target;
        bool check_for_obstacles(double target_x, double target_y);
        double extent_x;
        double extent_y;
        
    public:
        void move_forward(double target);
        void move_backward(double target);
        void turn_left(double target);
        void turn_right(double target);
        void move(int powerL, int powerR);
        void stop();
        void turn_to_heading(double target_heading);
        void goto_xy(double target_x, double target_y);
};