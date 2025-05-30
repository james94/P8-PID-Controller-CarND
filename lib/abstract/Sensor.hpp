#pragma once

class Sensor {
public:
    // Pure virtual function
    virtual void read() = 0;
    virtual ~Sensor() {}
};
