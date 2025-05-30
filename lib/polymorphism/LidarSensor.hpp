#pragma once
#include "abstract/Sensor.hpp"

class LidarSensor : public Sensor {
public:
    void read() override {
        // Lidar-specific reading logic
    }
};
