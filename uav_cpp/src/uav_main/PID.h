// PID.h
#ifndef PID_H
#define PID_H

class PID {
public:
    // Constructor to initialize the PID coefficients
    PID(double kp, double ki, double kd);

    // Method to calculate the control variable based on target and current values
    double calculate(double target, double current);

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
};

#endif // PID_H
