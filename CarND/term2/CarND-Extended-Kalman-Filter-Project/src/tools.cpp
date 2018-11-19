#include <iostream>
#include <math.h>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() {}


Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Checking the validity of the following inputs:
  //  * the estimation vector size should not be zero
  assert (estimations.size() != 0);
  //  * the estimation vector size should be equal ground truth vector size.
  assert (estimations.size() == ground_truth.size());

  VectorXd est_sum(4);
  est_sum << 0.0, 0.0, 0.0, 0.0;
  VectorXd est_squared(4);
  // Accumulate squared residuals.
  for(unsigned i = 0; i < estimations.size(); ++i){
    est_squared = estimations[i] - ground_truth[i];
    est_squared = est_squared.array() * est_squared.array();
    est_sum += est_squared;
  }

  //calculate the mean
  VectorXd denominator(4);
  float num = estimations.size();
  denominator << num, num, num, num;
  rmse = est_sum.array() /  denominator.array();
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  // Recover state parameters.
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px_py_squared_sum = px * px  + py * py;

  // Check if we are dividing by zero.
  if (fabs(px_py_squared_sum) < 0.001)
  {
      cout << "px^2 + py^2 is close to 0 ! Returning default value!";
      return Hj;
  }


  // Calculating individual partial derivatives.
  // First row.
  float d_rho_d_px = px / sqrt(px_py_squared_sum);
  float d_rho_d_py = py / sqrt(px_py_squared_sum);
  float d_rho_d_vx = 0;
  float d_rho_d_vy = 0;
  // Second row.
  float d_phi_d_px = -py / px_py_squared_sum;
  float d_phi_d_py = px / px_py_squared_sum;
  float d_phi_d_vx = 0;
  float d_phi_d_vy = 0;
  // Third row:
  float d_phd_d_px = py * (vx * py - vy * px) / pow(px_py_squared_sum, 1.5);
  float d_phd_d_py = px * (vy * px - vx * py) / pow(px_py_squared_sum, 1.5);
  float d_phd_d_vx = px / sqrt(px_py_squared_sum);
  float d_phd_d_vy = py / sqrt(px_py_squared_sum);

  // Populate the Jacobian matrix.
  Hj <<   d_rho_d_px, d_rho_d_py, d_rho_d_vx, d_rho_d_vy,
          d_phi_d_px, d_phi_d_py, d_phi_d_vx, d_phi_d_vy,
          d_phd_d_px, d_phd_d_py, d_phd_d_vx, d_phd_d_vy;

  return Hj;
}


MatrixXd Tools::CreateProcessCovariance(const float& dt, const float& ax2, const float& ay2) {
  MatrixXd Q(4,4);
  float t_4 = pow(dt, 4.0) / 4.0;
  float t_3 = pow(dt, 3.0) / 2.0;
  float t_2 = pow(dt, 2.0);
  Q <<  t_4 * ax2 ,       0.0, t_3 * ax2,        0.0,
        0.0       , t_4 * ay2,       0.0,  t_3 * ay2,
        t_3 * ax2 ,       0.0, t_2 * ax2,        0.0,
        0.0       , t_3 * ay2,       0.0,  t_2 * ay2;
  return Q;
}


VectorXd Tools::Convert2Polar(const VectorXd& x) {
  VectorXd z_pred(3);
  z_pred << 0.0, 0.0, 0.0;

  // Recover state parameters.
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  float px_py_squared_sum = px * px + py * py;

  // Check if rho is close to zero as division by rho is rho-rate.
  if (sqrt(px_py_squared_sum) < 0.001)
  {
      cout << "px^2 + py^2 is close to 0 ! Returning default value!";
      return z_pred;
  }

  float rho = sqrt(px_py_squared_sum);

  float phi = atan2(py , px);

  float delta_rho = 0;
  delta_rho = (px * vx + py * vy) / rho;

  z_pred << rho, phi, delta_rho;

  return z_pred;
}

