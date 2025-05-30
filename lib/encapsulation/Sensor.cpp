#include "Sensor.hpp"

Sensor::Sensor(const std::string& sensorId) 
    : id(sensorId), lastReading(0.0) { }

void Sensor::read() {
    // Simulate reading sensor data
    lastReading = 42.0;
}

double Sensor::getLastReading() const {
    return lastReading;
}

std::string Sensor::getId() const {
    return id;
}
