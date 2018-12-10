#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#define _USE_MATH_DEFINES
#include <math.h>

#include "Twiddle.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Normalize steering angle.
double normalize_steering_angle(double steering_angle) {
  if (steering_angle < -1.0)
    steering_angle = 1.0;
  else if (steering_angle > 1.0)
    steering_angle = 1.0;
  return steering_angle;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  //std::vector<double> p(3);
  ////p = Twiddle::TwiddleSteering(3800); // 3800 ~= one lap
  //p = Twiddle::TwiddleSteering(1000);
  //std::cout << "p: (" << p[0] << "," << p[1] << "," << p[2] << ")" << std::endl;
  //return 0;

  uWS::Hub h;

  PID   pid_steering;
  //pid_steering.Init(0.2, 0.004, 3.0); // Values from Udacity PID lessons.
  pid_steering.Init(0.15, 0.001, 3.5);
  PID   pid_throttle;
  pid_throttle.Init(0.25, 0.000001, 0.025); // Assume there is essentially no bias in the throttle, Ki ~= 0.0

  h.onMessage([&pid_steering,&pid_throttle](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
           * TODO: Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe use
           * another PID controller to control the speed!
           */
          pid_steering.UpdateError(cte);
          double  steer_value = normalize_steering_angle(pid_steering.TotalError());

          pid_throttle.UpdateError(cte);
            // Add a bit of throttle on top of the PID value to prevent backward
            // motion at the beginning and to increase speed later on.
          double  throttle_value = 0.5 + pid_throttle.TotalError();

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          std::cout << msg << std::endl;
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
