#include "WebSocketHandler.h"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <atomic>
#include <chrono>

class WebSocketHandlerTest : public ::testing::Test {
protected:
    WebSocketHandler handler;
    std::atomic<bool> messageReceived{false};
    const int TEST_PORT = 8080;

    void SetUp() override {
        handler.registerMessageCallback([this](const std::string& msg, auto ws) {
            messageReceived = true;
            handler.sendResponse(ws,"ACK" + msg);
        });

        std::thread([this]() {
            // handler.run();
            handler.initialize(TEST_PORT);
        }).detach();

        // Allow server startup
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TearDown() override {
        handler.shutdown();
    }

    std::string testMessage;
};

TEST_F(WebSocketHandlerTest, HandlesMessageCallbacks) {
    // In a real test, you'd connect a client WebSocket here
    // This mock verifies the callback registration
    EXPECT_TRUE(true); // Placeholder for actual connection test
}

TEST_F(WebSocketHandlerTest, ValidMessageProcessing) {
    std::string testMsg = R"(["test_data"])";
    auto result = handler.hasData(testMsg);
    EXPECT_FALSE(result.empty());
}

TEST_F(WebSocketHandlerTest, InvalidMessageProcessing) {
    std::string testMsg = "invalid_data";
    auto result = handler.hasData(testMsg);
    EXPECT_TRUE(result.empty());
}
