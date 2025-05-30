#pragma once
#include "abstract/Controller.hpp"

class PID : public Controller {
private:
    // PID coefficients
    double Kp, Ki, Kd;
    // Errors
    double p_error, i_error, d_error;

public:
    // Constructor
    PID(double kp, double ki, double kd);

    // Update errors based on cross track error (CTE)
    void UpdateError(double cte) override;

    // Calculate steering value
    double UpdateSteering() const override;
};
