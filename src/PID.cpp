#include <limits>
#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->prev_cte = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  this->p_error += cte * this->Kp;
  this->i_error += cte * this->Ki;
  this->d_error += cte * this->Kd;
}

double PID::TotalError() {
  return this->p_error + this->i_error + this->d_error;
}

double PID::Value(double cte) {
  // Calculate delta CTE.
  double  diff_cte =
    (std::numeric_limits<double>::min() == prev_cte) ? 0.0 : cte - prev_cte;
  prev_cte = cte;

  // Add current CTE to the total.
  UpdateError(cte);

  // PID formula
  return -Kp * cte - Kd * diff_cte - Ki * TotalError();
}
