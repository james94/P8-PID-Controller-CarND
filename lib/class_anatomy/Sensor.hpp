#pragma once

class Sensor {
public:
    std::string id;
    double lastReading;

    void read();
};