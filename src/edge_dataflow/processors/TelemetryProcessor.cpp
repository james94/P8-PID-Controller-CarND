
#include "TelemetryProcessor.h"

void TelemetryProcessor::process(const json& telemetry) {
    current_state_.cte = JsonUtils::parseWithDefault(telemetry, "cte", 0.0);
    current_state_.speed = telemetry["speed"];
    current_state_.steering_angle = telemetry["steering_angle"];

    steering_pid_.updateError(current_state_.cte);
    throttle_pid_.updateError(calculateSpeedError());
}

json TelemetryProcessor::generateResponse() {
    return JsonUtils::createResponse(
        steering_pid_.calculateOutput(),
        throttle_pid_.calculateOutput()
    );
}
