#include <iostream>
#include <polymorphism/LidarSensor.hpp>

int main() {
    LidarSensor frontLidar;
    
    try {
        frontLidar.read();
    } catch (const std::exception& e) {
        std::cerr << "Sensor error: " 
                  << e.what() << std::endl;
    }

    return 0;
}
