/*
##
# Data Processor Class (TelemetryProcessor)
##

- Contains both PID controllers
- Manages vehicle state data
- Implements business logic for throttle/steering calculations
- Handles JSON data conversion


*/

class TelemetryProcessor {
public:
    struct VehicleState {
        double cte;
        double speed;
        double steering_angle;
    };

    void process(const json& telemetry);
    json generateResponse();

private:
    PIDProcessor steering_pid_;
    PIDProcessor throttle_pid_;
    VehicleState current_state_;

    double calculateSpeedError();
};
