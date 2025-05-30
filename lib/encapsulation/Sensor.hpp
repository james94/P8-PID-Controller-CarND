#pragma once

class Sensor {
private:
    std::string id;
    double lastReading;

public:
    Sensor(const std::string& sensorId);

    void read();

    double getLastReading() const;

    std::string getId() const;
};
