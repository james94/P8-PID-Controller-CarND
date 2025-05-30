#pragma once
#include "encapsulation/Sensor.hpp"

class LidarSensor : public Sensor {
public:
    LidarSensor(const std::string& sensorId) 
        : Sensor(sensorId) {}

    void calibrate() {
        // Lidar-specific calibration logic
    }
};
