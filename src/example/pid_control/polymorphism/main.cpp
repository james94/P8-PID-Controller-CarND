#include <iostream>
#include <vector>
#include "inheritance/PID.hpp"

int main() {
    std::vector<Controller*> controllers;
    // Steering PID
    controllers.push_back(new PID(0.2, 0.004, 3.0)); 
    // Could add a Throttle PID 
    // or even a Deep Learning CNNController

    for (auto* ctrl : controllers) {
        // Calls the correct read() for each sensor type
        ctrl->UpdateError(/* value from telemetry */);
        double output = ctrl->GetControlValue();
        // Use output for steering or throttle
    }
    return 0;
}
