#pragma once
#include <cmath>
constexpr double FRONT_TO_BACK_WHEEL_DISTANCE_IN = 0;
constexpr double SIDE_TO_SIDE_WHEEL_DISTANCE_IN = 0;
constexpr double SIDE_TO_TRACKING_CENTER_IN = 0;
constexpr double FRONT_TO_TRACKING_CENTER_IN = 0;
constexpr double BACK_TO_TRACKING_CENTER_IN = 0;
constexpr double RAD_TO_DEG = 180 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180;
constexpr double GEAR_RATIO = 60/36;
constexpr double WHEEL_DIAMETER_IN = 2.75;
constexpr double DRIVE_DIST_PER_DEG = WHEEL_DIAMETER_IN * M_PI / 360 / GEAR_RATIO;

//This is found by spinning the robot 3 times and dividing the expected value (1080 degrees) by the actual value.
constexpr double IMU_DRIFT_SCALE_FACTOR = 1080 / (2*360 + 344.56); 