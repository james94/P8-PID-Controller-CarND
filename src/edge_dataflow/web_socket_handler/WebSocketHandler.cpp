#include "WebSocketHandler.h"
#include <iostream>

struct WebSocketHandler::Impl {
    uWS::Hub hub;
    int port = 0;
    MessageCallback callback;
    bool running = false;

    Impl() {
        hub.onConnection([this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
            std::cout << "Client connected" << std::endl;
        });
        
        hub.onDisconnection([this](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
            std::cout << "Client disconnected" << std::endl;
        });
    }

};

WebSocketHandler::WebSocketHandler() : impl_(std::make_unique<Impl>()) {

}

WebSocketHandler::~WebSocketHandler() = default;

void WebSocketHandler::initialize(int port) {
    impl_->port = port;
    impl_->hub.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        std::string message(data, length);
        auto s = hasData(message);
        if(!s.empty() && impl_->callback) {
            impl_->callback(s, ws);
        }
    });

    if(!impl_->hub.listen(port)) {
        std::cerr << "Failed to listen on port " << port << std::endl;
        return;
    }
    impl_->hub.run();
}

std::string WebSocketHandler::hasData(const std::string& s) const {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[]");
    auto b2 = s.find_last_of("]");

    if(found_null != std::string::npos) {
        return "";
    }

    if(b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }

    return "";
}

void WebSocketHandler::registerMessageCallback(MessageCallback callback) {
    impl_->callback = callback;
}

void WebSocketHandler::sendResponse(uWS::WebSocket<uWS::SERVER> ws, const std::string& message) {
    ws.send(message.data(), message.size(), uWS::OpCode::TEXT);
}

void WebSocketHandler::shutdown() {
    impl_->hub.getDefaultGroup<uWS::SERVER>().close();
}
