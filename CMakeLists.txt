cmake_minimum_required (VERSION 3.20)
project(PID_WebSocket_Controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Conan dependencies
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(libuv REQUIRED)
find_package(GTest REQUIRED CONFIG)
find_package(uwebsockets REQUIRED)

# Main Library Components
add_library(EdgeDataflowProcessors
    src/edge_dataflow/web_socket_handler/WebSocketHandler.cpp
)

target_include_directories(EdgeDataflowProcessors PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/edge_dataflow/web_socket_handler
    ${uwebsockets_INCLUDE_DIRS}
)

target_link_libraries(EdgeDataflowProcessors PUBLIC
    nlohmann_json::nlohmann_json
    openssl::openssl
    uv_a
    uwebsockets::uwebsockets
)

# Main executable with WebSocket test
# add_executable(WebSocketMain
#     src/main.cpp
# )

# target_link_libraries(WebSocketMain PRIVATE
#     EdgeDataflowProcessors
# )

# Test Configuration
enable_testing()
add_executable(ControllerTests
    tests/unit/WebSocketTest.cpp
)

message("GTest_LIBRARIES = ${GTest_LIBRARIES}")
target_link_libraries(ControllerTests PUBLIC
    EdgeDataflowProcessors
    GTest::gtest_main
    uwebsockets::uwebsockets
)

include(GoogleTest)
gtest_discover_tests(ControllerTests)
