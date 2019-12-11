#ifndef PID_H
#define PID_H

#include <string>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Parameter Constructor
   */
  PID(std::string control_name_);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Calculate the steering value
   * @output The steering value
   */
  double UpdateSteering();

  /**
   * Calculate the throttle value
   * @output The throttle value
   */
  double UpdateThrottle();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * PID device control name
   */
  std::string control_name;
};

#endif  // PID_H