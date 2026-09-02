#pragma once
class drivetrain{
    private:
        double target;
    public:
        void move_forward(double dist, int power);
        void move_backward(double dist, int power);
        void turn_left(double angle, int power);
        void turn_right(double angle, int power);
        void move(int powerL, int powerR);
        void stop();
};