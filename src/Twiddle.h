#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <uWS/uWS.h>
#include "PID.h"

class Twiddle
{
public:
  double  success_factor = 1.1; // 1.1 1.8
  double  failure_factor = 0.9; // 0.9 0.2

  PID&    pid;
  int     nparams;
  std::vector<double> p;  // parameters
  std::vector<double> dp; // deltas
  double  best_err;
  double  curr_err;
  int     counter;
  size_t  iparam;
  int     op;

public:
  Twiddle(PID& pid);
  ~Twiddle();

  void write();
  void write_err();
  void write_params();
  void write_deltas();

  bool next_param(double tolerance);
  void reset(uWS::WebSocket<uWS::SERVER>* ws);
  void update(uWS::WebSocket<uWS::SERVER>* ws, double tolerance);

  static std::vector<double> TwiddleSteering(int n = 100, double tolerance = 0.2);
};

#endif // TWIDDLE_H
