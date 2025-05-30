#include <iostream>
#include <vector>
#include <polymorphism/LidarSensor.hpp>
#include <polymorphism/CameraSensor.hpp>

int main() {
    std::vector<Sensor*> sensors;
    sensors.push_back(new LidarSensor("LIDAR_FRONT"));
    sensors.push_back(new CameraSensor("CAMERA_LEFT"));
    for (auto sensor : sensors) {
        // Calls the correct read() for each sensor type
        sensor->read(); 
    }
    return 0;
}
