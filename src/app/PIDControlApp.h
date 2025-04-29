
/*
##
# Main Application Class (PID Control App)
##

- Composes WebSocket and Telemetry Processors
- Implements application lifecycle management
- Manages cross-component communication

*/
class PIDControlApp {
public:
    void initialize();
    void run();

private:
    WebSocketHandler ws_handler_;
    TelemetryProcessor telemetry_processor_;

    void setupCallbacks();
};