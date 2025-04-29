

int main() {
    PIDControlApp app;

    // Configure PID parameters
    app.telemetry_processor_.configureSteeringPID(0.10, 0.0001, 1.0);
    app.telemetry_processor_.configureThrottlePID(0.1, 0.00015, 0.0);

    app.initialize();
    app.run();

    return 0;
}
