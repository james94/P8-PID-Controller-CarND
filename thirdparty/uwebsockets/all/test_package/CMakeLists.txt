cmake_minimum_required(VERSION 3.15)
project(test_package LANGUAGES CXX)

find_package(uwebsockets REQUIRED CONFIG)
find_package(OpenSSL REQUIRED)
find_package(libuv REQUIRED)

add_executable(${PROJECT_NAME} test_websocket.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE 
    uwebsockets::uwebsockets
    openssl::openssl
    uv_a
)

message("uwebsockets_INCLUDE_DIR = ${uwebsockets_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${uwebsockets_INCLUDE_DIR}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ASIO_STANDALONE
    _WEBSOCKETPP_CPP11_STL_
)
