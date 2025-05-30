#include <iostream>
#include <class_anatomy/PID.hpp>

int main() {
    PID steering_pid(0.2, 0.004, 3.0);
    // double cte = /* ideally get from simulator */;
    double cte = 0.001;
    return 0;
}
