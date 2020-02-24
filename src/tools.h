#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

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
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  static Eigen::MatrixXd compute_covariance_mat(float dt, float noise_ax, float noise_ay);
  static Eigen::MatrixXd compute_transition_mat(float time_difference);
  static Eigen::VectorXd convert_radial_to_cartesian(const Eigen::VectorXd &z);
  static Eigen::VectorXd convert_cartesian_to_radial(const Eigen::VectorXd &x);

};

#endif  // TOOLS_H_
