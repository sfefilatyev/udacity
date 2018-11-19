#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  noise_ax = 9;
  noise_ay = 9;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser.
  R_laser_ << 0.0225, 0.0000,
              0.0000, 0.0225;

  // Measurement covariance matrix - radar.
  R_radar_ << 0.09, 0.0000, 0.00,
              0.00, 0.0009, 0.00,
              0.00, 0.0000, 0.09;

  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  Hj_ << 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // First measurement.
    cout << "EKF: initializing..." << endl;

    VectorXd x_in = VectorXd(4);
    x_in << 0.0, 0.0, 0.0, 0.0;
    MatrixXd F_in = MatrixXd::Identity(4,4);
    F_in(0, 2) = 1.0; // This is not going to be used, as it is overwritten in the first prediction.
    F_in(1, 3) = 1.0; // This is not going to be used, as it is overwritten in the first prediction.
    // Initial confidence in the state is low. Thus, large values on the diagonal of state
    // covariance matrix.
    MatrixXd P_in = MatrixXd::Identity(4,4) * 1000.0;
    P_in(0, 0) = 1.0; // Position will be more certain thought.
    P_in(1, 1) = 1.0;

    MatrixXd R_in;
    MatrixXd Q_in = MatrixXd::Zero(4,4);
    MatrixXd H_in;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to Cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float delta_rho = measurement_pack.raw_measurements_[2];
      x_in(0) = rho * cos(phi);
      x_in(1) = rho * sin(phi);
      x_in(2) = delta_rho * cos(phi);
      x_in(3) = delta_rho * sin(phi);
      R_in = R_radar_;
      H_in = Hj_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_in(0) = measurement_pack.raw_measurements_[0];
      x_in(1) = measurement_pack.raw_measurements_[1];
      R_in = R_laser_;
      H_in = H_laser_;
    }

    ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);
    // Initializing time of the state vector.
    previous_timestamp_ = measurement_pack.timestamp_;
    // Done initializing, no need to predict or update.
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  // Updating time-measurement state of KF.
  previous_timestamp_ = measurement_pack.timestamp_;

  // Updating state transition matrix based on elapsed time. For brevity, only replacing matrix's
  // elements that depend on time.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Noise `ax` and `ay` are given as constants in this problem.
  // Updating process covariance matrix based on elapsed time. For readability using a tool's function.
  ekf_.Q_ = Tools::CreateProcessCovariance(dt, noise_ax, noise_ay);
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates. Conversion of the predicted state vector into polar coordinates
    // happens inside `UpdateEKF`.
    VectorXd z(3);
    float rho = measurement_pack.raw_measurements_[0];
    float phi = measurement_pack.raw_measurements_[1];
    float delta_rho = measurement_pack.raw_measurements_[2];
    z << rho, phi, delta_rho;
    Hj_ = Tools::CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);

  } else {
    // Laser updates.
    VectorXd z(2);
    z(0) = measurement_pack.raw_measurements_[0];
    z(1) = measurement_pack.raw_measurements_[1];
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);

  }

  // Print the output of the state and its covariance.
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
