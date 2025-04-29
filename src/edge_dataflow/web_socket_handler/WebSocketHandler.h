#pragma once
#include <uWS/uWS.h>
#include <functional>
#include <string>
#include <memory>

/*
##
# Network Communication Class (WebSocketHandler)
##

- Manages WebSocket connections and message routing
- Implements `hasData()` for message validation
- Handles connection/disconnection events
- Provides callback registration for message processing

*/

class WebSocketHandler {
public:
    // Use WebSocket by value (uWS API requirement)
    using MessageCallback = std::function<void(const std::string&, uWS::WebSocket<uWS::SERVER>)>;

    WebSocketHandler();
    ~WebSocketHandler();

    void initialize(int port);
    void registerMessageCallback(MessageCallback callback);
    void sendResponse(uWS::WebSocket<uWS::SERVER> ws, const std::string& message);
    void shutdown();

    // Message processing helper
    std::string hasData(const std::string& s) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
