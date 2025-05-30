#pragma once

class PID {
public:
    // PID coefficients
    double Kp, Ki, Kd;
    // Errors
    double p_error, i_error, d_error;

    // Constructor
    PID(double kp, double ki, double kd);

    // Update errors based on cross track error (CTE)
    void UpdateError(double cte);

    // Calculate steering value
    double UpdateSteering();
};
