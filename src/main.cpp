#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Initialize the pid variable for steering and speed.
  PID pid, pid_speed;

  // Increase P gain until the response to a disturbance is steady oscillation
  // Increase D gain until the oscillations go away
  // Increase I gain until it brings you to setpoint with number of oscillations desired

  // tau_p is kept small, so steering doesn't oscillate too much just enough for smooth steering
  // tau_i is kept small, allowing the car to steer from one side to the other in longer durations
  // tau_d is kept small, but 10x larger than tau_p, so oscillations go away
  // hence, less oscillations, means less aggressive steering
  pid.Init(0.10, 0.0001, 1.0); 

  // tau_p is kept small, so throttle doesn't oscillate too much just enough for gradual acceleration increase
  // tau_p is kept small for slower throttle oscillation, speed increases at a slow rate
  // tau_d is kept at 0 since we want oscillations to increase for reaching higher car speeds
  // less oscillations, means the throttle uses less engine power, so slower acceleration, slower speed increase 
  pid_speed.Init(0.1, 0.00015, 0.0);

  h.onMessage([&pid, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      try {

        auto s = hasData(string(data).substr(0, length));

        if (!s.empty()) {
          auto j = json::parse(s);
  
          string event = j[0].get<string>();
  
          std::cout << "Received event = " << event << std::endl;
  
          if (event == "telemetry") {
            // Add this debug line FIRST
            // std::cout << "Raw Telemetry Data: " << j[1].dump() << std::endl;

            // Modify validation to match actual Unity data
            if(j[1].find("steering_angle") == j[1].end() ||
               j[1].find("throttle") == j[1].end() ||
               j[1].find("speed") == j[1].end()) {
                throw std::runtime_error("Missing required telemetry fields");
            }

            // j[1] is the data JSON object
            // double cte = j[1]["cte"].get<double>();
            double cte = j[1].value("cte", 0.0); // Default to 0.0 if missing
            double speed = j[1]["speed"].get<double>();
            double angle = j[1]["steering_angle"].get<double>();
            double steer_value;
            double throttle_value;
  
            // Calculate steering value within [-1, 1]
            pid.UpdateError(cte);
            steer_value = pid.UpdateSteering();
  
            // Calculate throttle value within [0, 1], which controls car's speed
            double desired_speed = 30;
            double speed_err = abs(desired_speed - speed)/desired_speed;
            pid_speed.UpdateError(speed_err);
            throttle_value = pid_speed.UpdateThrottle();
  
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << std::endl;
  
            json msgJson;
            // msgJson["steering_angle"] = steer_value;
            // msgJson["throttle"] = throttle_value; // trying with pid speed
            
            // Data Formatting Adjustments for Precision Control & Type Consistency
            msgJson["steering_angle"] = std::round(steer_value * 1000.0) / 1000.0; // 3 decimal places
            msgJson["throttle"] = std::round(throttle_value * 1000.0) / 1000.0; // 3 decimal places

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << "Sending: " << msg << std::endl;
            // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            // Add async send verification
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT,
                [](void*, void*, bool cancelled, void*) {
                    if(!cancelled) {
                      std::cerr << "Failed to send steer command!" << std::endl;
                    }
                },
                nullptr); // Last parameter is callback user data
          }  // end "telemetry" if
        } else {
          // Manual driving
          string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

      } catch (const std::exception& e) {
        std::cerr << "Error processing message: " << e.what() << std::endl;
        // send error response
        auto msg = "42[\"error\",{\"message\":\"" + string(e.what()) + "\"}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }

    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}