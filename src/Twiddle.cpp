#include "Twiddle.h"

#include <iostream>
#include <numeric>

#include "json.hpp"
using json = nlohmann::json;

extern double normalize_steering_angle(double steering_angle);  // Defined in main.cpp
extern std::string hasData(std::string s);                      // Defined in main.cpp

Twiddle::Twiddle(PID& _pid)
  : pid(_pid)
{
  nparams  = 3;
  best_err = std::numeric_limits<double>::max();
  curr_err = 0.0;
  counter  = 0;
  iparam   = 0;
  op       = 0;

  p.resize(nparams);
  dp.resize(nparams);
  std::fill(p.begin(), p.end(), 0.0);
  std::fill(dp.begin(), dp.end(), 1.0);

  pid.Init(p[0], p[1], p[2]);

  // Tweak a couple of values so we'll get back to the desired starting
  // point when best_err is set for the first time, i.e., with iparam
  // set to 0 and all dp values set to 1.0
  iparam = nparams - 1;
  dp[iparam] /= success_factor;
}

Twiddle::~Twiddle()
{
}

void Twiddle::write() {
  std::cout << "op: " << op << " i: " << iparam << std::endl;
  write_err();
  write_params();
  write_deltas();
}

void Twiddle::write_err() {
  std::cout << "best_err: " << best_err << " curr_err: " << curr_err << std::endl;
}

void Twiddle::write_params() {
  std::cout << "params: (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
  std::cout << "   pid: (" << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << ")" << std::endl;
}

void Twiddle::write_deltas() {
  std::cout << "deltas: (" << dp[0] << ", " << dp[1] << ", " << dp[2] << ") -> "
            << std::accumulate(dp.begin(), dp.end(), 0.0) << std::endl;
}

bool Twiddle::next_param(double tolerance) {
  bool _continue = true;

  //std::cout << "next_param: reset op, curr_err to 0.0" << std::endl;
  op = 0;         // Reset the op to addition for the next parameter.
  curr_err = 0.0; // Reset the current error.

  //std::cout << "next_param: iparam: " << iparam << " -> " << (iparam + 1) % p.size() << std::endl;
  if (++iparam == p.size()) {
    iparam = 0; // Reset to first parameter.

    // Check our tolerance after each pass through all parameters.
    // Keep going until we've reduced the accumulated parmeter deltas to
    // the specified tolerance.
    _continue = std::accumulate(dp.begin(), dp.end(), 0.0) > tolerance;
  }

  return _continue;
}

// Reset for another test run.
#ifdef _WIN32
void Twiddle::reset(uWS::WebSocket<uWS::SERVER>* ws) {
#else
void Twiddle::reset(uWS::WebSocket<uWS::SERVER> ws) {
#endif
  counter = 0;
  pid.Init(p[0], p[1], p[2]);

  std::string msg = "42[\"reset\",{}]";
#ifdef _WIN32
  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
}

#ifdef _WIN32
void Twiddle::update(uWS::WebSocket<uWS::SERVER>* ws, double tolerance) {
#else
void Twiddle::update(uWS::WebSocket<uWS::SERVER> ws, double tolerance) {
#endif

  std::cout << "----------------------------------------" << std::endl;
  std::cout << ">> update" << std::endl;
  write_params();
  write_deltas();
  std::cout << "----------" << std::endl;

  bool  shutdown = false;

  // Update best error?
  if (curr_err < best_err) {
    std::cout << "update best error" << std::endl;
    write_err();
    best_err = curr_err;
    dp[iparam] *= success_factor;
    if (!next_param(tolerance)) {
      ;//return twiddle.p;
      write_params();
      shutdown = true;
    }
  }

  // Try the next operation (add then subtract) or move on to the next parameter.
  else {
    switch (op++) {
      case 0:   // Add
        std::cout << "update p[" << iparam << "]" << std::endl;
        p[iparam] += dp[iparam];
        break;
      case 1:   // Subtract
        std::cout << "update p[" << iparam << "]" << std::endl;
        p[iparam] -= (2.0 * dp[iparam]);
        break;
      default:  // Reset
        std::cout << "reset" << std::endl;
        p[iparam] += dp[iparam];
        dp[iparam] *= failure_factor;
        if (!next_param(tolerance)) {
          ;//return p;
          write_params();
          shutdown = true;
        }
        break;
    }
  }

  // Reset for another twiddle run.
  reset(ws);

  std::cout << "----------" << std::endl;
  write_params();
  write_deltas();
  std::cout << "<< update" << std::endl;
  std::cout << "----------------------------------------" << std::endl;

#ifdef _WIN32
  if (shutdown)
    ws->shutdown();
#endif
}

std::vector<double> Twiddle::TwiddleSteering(int n, double tolerance) {

  PID     pid_steering;
  Twiddle twiddle(pid_steering);

  uWS::Hub h;

#ifdef _WIN32
  h.onMessage([&pid_steering,&twiddle,&n,&tolerance](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
#else
  h.onMessage([&pid_steering,&twiddle,&n,&tolerance](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif
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

          //std::cout << "CTE: " << cte << " Speed: " << speed << " SteeringAngle: " << angle << std::endl;

          pid_steering.UpdateError(cte);
          double  steer_value = normalize_steering_angle(pid_steering.TotalError());

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
#ifdef _WIN32
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif

          // DEBUG
          //if (twiddle.counter % 100 == 0)
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          ++twiddle.counter;
          if (twiddle.counter >= 150) // Give the PID controller a chance to converge.
            twiddle.curr_err += pow(cte, 2);

          constexpr double max_cte = 6.0;
          bool  off_road = cte > max_cte || speed <= 0.01;
          // If we've reached the specified number of iterations for twiddle *or*
          // if the current CTE is > max_cte then update twiddle params as necessary
          // and move to the next op (add/subtract) or the next parameter.
          // Note: Assume that a CTE > max_cte means that the car has run off the road
          // and therefore we end the current twiddle run and reset.
          if (twiddle.counter >= n || off_road) {
            twiddle.curr_err /= n; // Normalize error

            //std::cout << "----------------------------------------" << std::endl;
            //std::cout << ">> begin twiddle update: " << std::endl;
            //twiddle.write();
            //std::cout << "----------" << std::endl;

            twiddle.update(ws, tolerance);

            //std::cout << "----------" << std::endl;
            //twiddle.write();
            //std::cout << ">> end twiddle update" << std::endl;
            //std::cout << "----------------------------------------" << std::endl;
          } // end of twiddle run
        } // end if telemetry
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef _WIN32
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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

#ifdef _WIN32
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef _WIN32
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
    ws->close();
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef _WIN32
  auto host = "127.0.0.1";
  if (h.listen(host, port))
#else
  if (h.listen(port))
#endif
  {
    std::cout << "Listening to port " << port << std::endl;
    h.run();  // There is no escape from this call - program termination occurs when the socket is closed.
              // What up with that?!
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
  }

  std::cout << "Returning" << std::endl;
  return twiddle.p;
}


/*
 * Generic twiddle algorithm. This will not work with the CarND term2 simulator since
 * it does not appear to be possible to implement an equivalent of the "run" function.
 * The entire remainder of this class is an attempt to implement this algorithm within
 * the restrictions imposed by using the simulator. The following code is included only
 * for reference.
 */
std::vector<double> Twiddle(int nparams, double tolerance = 0.2) {

  std::vector<double> p(nparams);   // parameters
  std::vector<double> dp(nparams);  // deltas

  std::fill(p.begin(), p.end(), 0.0);
  std::fill(dp.begin(), dp.end(), 1.0);

  double  best_err = 0.0; //Run(p);

  double  success_factor = 1.1; // 1.8
  double  failure_factor = 0.9; // 0.2

  // Loop until we reach the specified tolerance.
  while (std::accumulate(dp.begin(), dp.end(), 0.0) > tolerance) {
    // For each parameter ...
    for (size_t i = 0; i < p.size(); ++i) {
      // First try adding.
      p[i] += dp[i];
      double  err = 0.0; //Run(p);
      if (err < best_err) {
        best_err = err;
        dp[i] *= success_factor;
      }
      else {
        // Adding did not improve the result, try subtracting.
        // Note that we subtract twice because we already added above.
        p[i] -= 2.0 * dp[i];
        double  err = 0.0; //Run(p);
        if (err < best_err) {
          best_err = err;
          dp[i] *= success_factor;
        }
        else {
          // Neither adding nor subtracting improved the result.
          // Reset the parameter and adjust the delta.
          p[i] += dp[i];
          dp[i] *= failure_factor;
        }
      }
    } // end for each parameter
  } // end while not at tolerance

  std::cout << "Best error: " << best_err << std::endl;

  return p;
}
