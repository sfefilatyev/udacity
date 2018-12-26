#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // Measurement noise covariance matrix for radar
  MatrixXd R_radar_;

  // Measurement noise covariance matrix for lidar
  MatrixXd R_lidar_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  unsigned n_x_;

  ///* Augmented state dimension
  unsigned n_aug_;

  // Radar measurement dimension
  unsigned n_z_;

  // Lidar measurement dimension
  unsigned n_l_;

  ///* Sigma point spreading parameter
  double lambda_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package: Reference to the latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage & meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement.
   * @param meas_package: Reference to the measurement at k+1 from lidar.
   */
  void UpdateLidar(const MeasurementPackage & meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement.
   * @param meas_package: Reference to the measurement at k+1 from radar.
   */
  void UpdateRadar(const MeasurementPackage & meas_package);

private:
  /**
   * Populates provided matrix with sigma points based on the current state and covariance.
   * @param reference to MatrixD matrix that will store results.
   */
  void GenerateSigmaPoints(MatrixXd & Xsig_aug);

  /**
   * Populates provided matrix with predicted sigma points.
   * @param Xsig_aug: constant reference to initial sigma points matrix.
   *        Xsig_pred: reference to MatrixD matrix that will store predicted sigma points.
   *        delta_t: time difference between measurements.
   */
  void PredictSigmaPoints(const MatrixXd & Xsig_aug, MatrixXd & Xsig_pred, float delta_t);

  /**
   * Predict mean state and its covariance matrix.
   * @param: Xsig_pred: reference to predicted sigma points matrix.
   */
  void PredictMeanAndCovariance(const MatrixXd & Xsig_pred);

  /**
   * Predict mean and covariance of the state vector in the radar measurement space
   * @param Xsig_pred: constant reference to MatrixD matrix that will store predicted sigma points.
   *        z_pred: reference to vector to hold predicted mean of the measurement.
   *        S: reference to a covariance matrix of the measurement.
   */
  void PredictRadarMeasurement(const MatrixXd & Xsig_pred, MatrixXd & Zsig, VectorXd & z_pred, MatrixXd & S);

};

#endif /* UKF_H */
