#pragma once
#include "abstract/Sensor.hpp"

class CameraSensor : public Sensor {
public:
    void read() override {
        // Camera-specific reading logic
    }
};
