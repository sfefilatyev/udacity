#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobian.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
   *  A helper method to create a process covariance matrix;
   */
  static MatrixXd CreateProcessCovariance(const float& dt, const float& ax2, const float& ay2);

  /**
   *  A helper method to convert state vector values to polar coordinates.
   */
  static VectorXd Convert2Polar(const VectorXd& x);

};

#endif /* TOOLS_H_ */
