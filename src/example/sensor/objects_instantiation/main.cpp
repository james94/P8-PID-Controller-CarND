#include <iostream>
#include <class_anatomy/Sensor.hpp>

int main() {
    Sensor frontLidar;
    frontLidar.id = "LIDAR_FRONT";
    frontLidar.read();
    return 0;
}
