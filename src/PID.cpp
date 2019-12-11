#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

/**
 * Init() function
 * 
 * Initializes PID coefficients and crosstrack error(s)
 */
void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID Coefficients tau Kp, Ki, Kd
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // Initialize integral crosstrack error
  i_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}