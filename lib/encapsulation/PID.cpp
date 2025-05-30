#include "PID.hpp"

// Constructor
PID::PID(double kp, double ki, double kd) 
    : Kp(kp), Ki(ki), Kd(kd),
      p_error(0), i_error(0), d_error(0) 
{

}

// Update errors based on cross track error (CTE)
void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

// Calculate steering value
double PID::UpdateSteering() const {
    return -Kp * p_error - Ki * i_error - Kd * d_error;
}