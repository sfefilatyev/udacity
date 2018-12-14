#include "ukf.h"
#include "Eigen/Dense"
#include <math.h>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  x_ << 0.0, 0.0, 0.0, 0.0, 0.0;

   ///* state covariance matrix
  P_ << 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0;

  time_us_ = 0;
  is_initialized_ = false;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = n_x_ + 2;

  ///* Sigma point spreading parameter
  lambda_= 3;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // First measurement.
    cout << "UKF: initializing..." << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to Cartesian coordinates and initialize state.
      */
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float delta_rho = meas_package.raw_measurements_[2];
      /*
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      x_(2) = delta_rho * cos(phi);
      x_(3) = delta_rho * sin(phi);
      */

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];

    }

    // Initializing time of the state vector.
    time_us_ = meas_package.timestamp_;
    // Done initializing, no need to predict or update.
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  // Updating time-measurement state of KF.
  time_us_ = meas_package.timestamp_;

  // Updating state transition matrix based on elapsed time. For brevity, only replacing matrix's
  // elements that depend on time.

  // Noise `ax` and `ay` are given as constants in this problem.
  // Updating process covariance matrix based on elapsed time. For readability using a tool's function.
  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates.
    UpdateRadar(meas_package);
  } else {
    // Laser updates.
    UpdateLidar(meas_package);
  }

  // Print the output of the state and its covariance.
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  MatrixXd Xsig_aug, Xsig_pred;
  GenerateSigmaPoints(&Xsig_aug);
  PredictSigmaPoints(Xsig_aug, &Xsig_pred, delta_t);
  PredictMeanAndCovariance(Xsig_pred);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd z(2);
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  VectorXd z(3);
  float rho = meas_package.raw_measurements_[0];
  float phi = meas_package.raw_measurements_[1];
  float delta_rho = meas_package.raw_measurements_[2];
  z << rho, phi, delta_rho;

}

/**
 * Populates provided matrix with sigma points based on the current state and covariance.
 * @param pointer to MatrixD matrix that will store results.
 */
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
  // Set state dimension.
  int n_x = P_.cols();
  int n_aug = n_x + 2;

  // Create augmented mean vector.
  VectorXd x_aug  = VectorXd(n_aug);
  x_aug.head(n_x) = x_;
  x_aug(n_x) = 0;
  x_aug(n_x + 1) = 0;

  // Create augmented covariance matrix.
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x) = P_;
  P_aug(n_x, n_x) = std_a_ * std_a_;
  P_aug(n_x + 1, n_x + 1) = std_yawdd_ * std_yawdd_;

  // Define spreading parameter.
  double lambda = 3 - n_aug;

  // Create sigma point matrix.
  MatrixXd Xsig = MatrixXd(n_aug, 2 * n_aug + 1);

  // Calculate square root of P_aug.
  MatrixXd A = P_aug.llt().matrixL();

  // Actually placing the values inside the matrix.
  // 1. Assigning the mean vector as a first column.
  Xsig.col(0)  = x_aug;

  // 2. Assigning the values of the second and third group of sigmas.
  for (int i = 0; i < n_aug; i++)
  {
    Xsig.col(i + 1)     = x_aug + sqrt(lambda + n_aug) * A.col(i);
    Xsig.col(i + 1 + n_aug) = x_aug - sqrt(lambda + n_aug) * A.col(i);
  }

  // Write result.
  *Xsig_out = Xsig;
}

void UKF::PredictSigmaPoints(MatrixXd& Xsig_aug, MatrixXd* Xsig_pred, float delta_t){
  // Set state and augmented vector dimensions.
  int n_aug = Xsig_aug.cols();
  int n_x = n_aug - 2;
  MatrixXd Xsig = MatrixXd(n_x, n_x);
  Xsig.fill(0.0);

  // Predict sigma points.
  for (unsigned i = 0; i < n_aug; ++i) {
      float px = Xsig_aug(0, i);
      float py = Xsig_aug(1, i);
      float v = Xsig_aug(2, i);
      float phi = Xsig_aug(3, i);
      float phi_dot = Xsig_aug(4, i);
      float nu_a = Xsig_aug(5, i);
      float nu_phi = Xsig_aug(6, i);

      // Computing stochastic part common for both cases when phi is either 0 or not.
      float px_stochastic = pow(delta_t, 2) * cos(phi) * nu_a / 2;
      float py_stochastic = pow(delta_t, 2) * sin(phi) * nu_a / 2;
      float v_stochastic = delta_t * nu_a;
      float phi_stochastic = pow(delta_t, 2) * nu_phi / 2;
      float phi_dot_stochastic = delta_t * nu_phi;

      float px_gain = 0;
      float py_gain = 0;
      float v_gain = 0;
      float phi_gain = phi_dot * delta_t;
      float phi_dot_gain = 0;

      // Computing gain factor between two states. Avoiding division by zero.
      if (fabs(phi_dot) < 0.001) {
          px_gain = v * cos(phi) * delta_t;
          py_gain = v * sin(phi) * delta_t;
      }
      else {
          px_gain = v / phi_dot * (sin(phi + phi_dot * delta_t)  - sin(phi));
          py_gain = v / phi_dot * (-cos(phi + phi_dot * delta_t)  + cos(phi));
      }

      Xsig(0, i) = px + px_gain + px_stochastic;
      Xsig(1, i) = py + py_gain + py_stochastic;
      Xsig(2, i) = v + v_gain + v_stochastic;
      Xsig(3, i) = phi + phi_gain + phi_stochastic;
      Xsig(4, i) = phi_dot + phi_dot_gain + phi_dot_stochastic;
  }

  // Write result.
  *Xsig_pred = Xsig;
}

void UKF::PredictMeanAndCovariance(MatrixXd & Xsig_pred){

  //set state dimension
  int n_x = P_.cols();

  //set augmented dimension
  int n_aug = n_x + 2;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create vector for weights
  VectorXd weights = VectorXd(2 * n_aug + 1);

  // Create vector for predicted state.
  VectorXd x = VectorXd(n_x);
  x.fill(0.0);

  // Create covariance matrix for prediction.
  MatrixXd P = MatrixXd(n_x, n_x);
  P.fill(0.0);

  // Set weights.
  weights(0) = lambda / (lambda + n_aug);
  float weight_other = 1 / (lambda + n_aug) / 2;
  for (unsigned i = 1; i < Xsig_pred.cols(); ++i) {
      weights(i) = weight_other;
  }

  // Predict state mean.
  for (unsigned i = 0; i < Xsig_pred.cols(); ++i) {
      x += weights(i) * Xsig_pred.col(i);
  }

  // Predict state covariance matrix.
  for (unsigned i = 0; i < Xsig_pred.cols(); ++i) {
      VectorXd x_diff = Xsig_pred.col(i) - x;
      // Angle normalization.
      while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
      while (x_diff(3) <-M_PI) x_diff(3) += 2. * M_PI;
      P += weights(i) * x_diff * x_diff.transpose();
  }

  // Write result to existing state and covariance variables.
  x_ = x;
  P_ = P;
}
