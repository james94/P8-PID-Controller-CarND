#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <atomic>

bool test_websocket_features() {
    uWS::Hub h;
    std::atomic<bool> test_passed{false};
    std::atomic<bool> message_received{false};

    h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *message,
                    size_t length, uWS::OpCode opCode) {
        std::string msg(message, length);

        if(msg == "test_message") {
            message_received = true;
            ws.send("response", 8, opCode);
        }
    });

    h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        ws.send("test_message", 12, uWS::OpCode::TEXT);
    });

    h.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, int code,
                          char *message, size_t length) {
        test_passed = message_received.load();
    });

    if(!h.listen(3000)) {
        return false;
    }

    std::thread server_thread([&h]() {
        h.run();
    });

    // Client test
    uWS::Hub client_hub;
    client_hub.onMessage([&](uWS::WebSocket<uWS::CLIENT> ws, char *message,
                             size_t length, uWS::OpCode opCode) {
        if(std::string(message, length) == "response") {
            ws.close();
        }
    });

    client_hub.connect("ws://localhost:3000", nullptr);
    client_hub.run();

    server_thread.join();
    return test_passed.load();
}

int main() {
    if(!test_websocket_features()) {
        std::cerr << "WebSocket feature tests failed!" << std::endl;
        return 1;
    }
    std::cout << "All WebSocket tests passed!" << std::endl;
    return 0;
}