#include <iostream>
#include "inheritance/PID.hpp"

int main() {
    PID steering_pid(0.2, 0.004, 3.0);
    double cte = 0.001;
    
    try {
        steering_pid.UpdateError(cte);
    } catch (const std::exception& e) {
        std::cerr << "PID error: " 
                  << e.what() << std::endl;
    }

    return 0;
}
