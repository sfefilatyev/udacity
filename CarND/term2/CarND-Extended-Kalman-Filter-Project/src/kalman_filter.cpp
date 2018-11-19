#include "kalman_filter.h"
#include "tools.h"
#include "iostream"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Predict KF state and state covariance.
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Update the state and state covariance with a new measurement.
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd K = PHt * S.inverse();

  // Updating estimate.
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(4,4);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred(3);
  z_pred = Tools::Convert2Polar(x_);

  VectorXd y = z - z_pred;
  // Normalizing value of phi between -PI and PI.
  if (y(1) >= M_PI)
    y(1) = y(1) - 2 * M_PI;
  if (y(1) <= -M_PI)
    y(1) = y(1) + 2 * M_PI;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd K = PHt * S.inverse();

  // Updating estimate.
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(4,4);
  P_ = (I - K * H_) * P_;

}
