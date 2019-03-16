#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  iteration = 0;
}

void PID::UpdateError(double cte) {
  if (iteration == 0){
      cte_prev = cte;
  }

  p_error = cte;
  i_error += cte;
  d_error = cte - cte_prev;

  cte_prev = cte;
  ++iteration;
}

double PID::TotalError() {
  double total_error = Kp * p_error  + Ki * i_error + Kd * d_error;
  if (total_error > 1.0)
    total_error = 1.0;
  if (total_error < -1.0)
    total_error = -1.0;
  return total_error;
}

