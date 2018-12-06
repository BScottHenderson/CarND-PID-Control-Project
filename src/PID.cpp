#include <limits>
#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

bool  first_error = true;

void PID::Init(double _Kp, double _Ki, double _Kd) {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

void PID::UpdateError(double _cte) {
  d_error = (first_error) ? 0.0 : _cte - p_error;
  p_error = _cte;
  i_error += _cte;
  first_error = false;
}

double PID::TotalError() {
  // PID formula
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}
