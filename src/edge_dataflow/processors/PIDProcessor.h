
/*
##
# PID Controller Class (PIDProcessor)
##

- Generic PID implementation usable for both steering and throttle
- Separates error accumulation from output calculation
- Configurable gains through member functions
*/

class PIDProcessor {
public:
    void configure(double Kp, double Ki, double Kd);
    void updateError(double error);
    double calculateOutput();

    // Optional: Add reset functionality
    void reset();

private:
    double Kp_, Ki_, Kd_;
    double p_error_, i_error_, d_error_;
    double prev_error_;
};
