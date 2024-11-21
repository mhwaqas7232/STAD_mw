// PID.cpp
#include "PID.h"

PID::PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

double PID::calculate(double target, double current) {
    double error = target - current;
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;

    // PID output
    return (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
}
