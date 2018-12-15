#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

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
