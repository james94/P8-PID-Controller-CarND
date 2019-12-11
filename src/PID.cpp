#include "PID.h"
#include <iostream>

using std::cout;
using std::endl;

PID::PID() {}

/**
 * Sets name of device being controlled by PID Controller:
 * 
 * Device names: steering, throttle
 */
PID::PID(std::string control_name_)
{
  control_name = control_name_;
}

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

  // Initialize PID crosstrack errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_error = 0.0;
}


/**
 * UpdateError() function
 * 
 * Update PID errors based on cte.
 */
void PID::UpdateError(double cte) {
  // Update proportional error
  p_error = cte;

  // Update differential error
  d_error = p_error - prev_error;
  prev_error = p_error;

  // Update integral error
  i_error += p_error;
}

/**
 * TotalError() function
 * 
 * Calculate and return the total error for device being controlled:
 *
 * steering error or throttle error to control car's steering or
 * speed is returned.
 */
double PID::TotalError() {
  double total_error = 0.0;

  if(control_name == "steering")
  {
    total_error = UpdateSteering();
  }
  else if(control_name == "throttle")
  {
    total_error = UpdateThrottle();
  }
  else
  {
    std::cout << "Name of device to be controlled unknown" << std::endl;
  }
  
  return total_error; 
}

/**
* UpdateSteering() function
*
* Calculates the steering value
*/
double PID::UpdateSteering() {
  double steering_value = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
  
  // Ensure the steering value is [-1, 1]
  if(steering_value < -1)
  {
    steering_value = -1;
  }
  else if(steering_value > 1)
  {
    steering_value = 1;
  }

  return steering_value;
}

/**
* UpdateThrottle() function
*
* Calculates the throttle value
*/
double PID::UpdateThrottle() {
  double throttle_value = (Kp * p_error) + (Kd * d_error) + (Ki * i_error);
  
  // Ensure the throttle value is [0, 1]
  if(throttle_value < 0)
  {
    throttle_value = 0;
  }
  else if(throttle_value > 1)
  {
    throttle_value = 1;
  }

  return throttle_value;
}